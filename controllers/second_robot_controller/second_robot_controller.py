"""second_robot_controller controller."""

from typing import Any
import numpy as np
from controller import Robot


MAX_VELOCITY = 14.81

KUKA_WHEEL_RADIUS = 0.05
KUKA_WIDTH = 0.302
KUKA_LENGTH = 0.56

Map = {
    (1, 0): None,
    (1, 1): None,
    (2, 0): None,
    (2, 1): None,
}

# PID Factors
KP = 0.01
KD = 0.01
KI = 0

# Last error to be used by the PID.
last_error = 0

# Integral (the accumulation of errors) to be used by the PID.
integral = 0


def range_conversion(s_start: float, s_end: float, d_start: float, d_end: float, value: float):

    ratio = abs((value - s_start) / (s_end - s_start))

    if d_start < d_end:
        return d_start + abs(d_end - d_start) * ratio

    return d_start - abs(d_end - d_start) * ratio


class RobotController(Robot):

    def __init__(self):

        Robot.__init__(self)

        self.has_box = False

        self.has_box_arrived = [False, False, False, False]

        self.intersection = 1

        self.branch = -1

        self.time_step = int(self.getBasicTimeStep())

        self.front_right_wheel: Any = self.getDevice("wheel1")
        self.front_left_wheel: Any = self.getDevice("wheel2")
        self.back_right_wheel: Any = self.getDevice("wheel3")
        self.back_left_wheel: Any = self.getDevice("wheel4")

        self.front_right_wheel.setPosition(float("inf"))
        self.front_left_wheel.setPosition(float("inf"))
        self.back_right_wheel.setPosition(float("inf"))
        self.back_left_wheel.setPosition(float("inf"))

        self.front_right_wheel.setVelocity(0)
        self.front_left_wheel.setVelocity(0)
        self.back_right_wheel.setVelocity(0)
        self.back_left_wheel.setVelocity(0)

        self.arm2: Any = self.getDevice("arm2")
        self.arm3: Any = self.getDevice("arm3")
        self.arm4: Any = self.getDevice("arm4")
        self.left_finger: Any = self.getDevice("finger::left")

        self.left_finger.setPosition(0.025)
        self.arm2.setPosition(0)
        self.arm3.setPosition(0)
        self.arm4.setPosition(0)

        self.center_sensor: Any = self.getDevice("center sensor")
        self.center_sensor.enable(self.time_step)

        self.camera: Any = self.getDevice("camera")
        self.camera.enable(self.time_step)

        self.wall_sensor: Any = self.getDevice("wall sensor")
        self.wall_sensor.enable(self.time_step)

        self.sensors: list[Any] = list(
            map(lambda v: self.getDevice(f"lfs{v}"), range(8))
        )

        self.weights = [-1000, -1000, -1000, -1000, 1000, 1000, 1000, 1000]

        for sensor in self.sensors:
            sensor.enable(self.time_step)

        self.step(self.time_step)

        self.color: list[Any] = []

    def set_motors_velocity(
        self,
        wheel1_v: float,
        wheel2_v: float,
        wheel3_v: float,
        wheel4_v: float
    ):
        self.front_right_wheel.setVelocity(wheel1_v)
        self.front_left_wheel.setVelocity(wheel2_v)
        self.back_right_wheel.setVelocity(wheel3_v)
        self.back_left_wheel.setVelocity(wheel4_v)

    def get_sensors_value(self):

        value = 0

        for index, sensor in enumerate(self.sensors):
            if sensor.getValue() > 600:
                value += self.weights[index]

        return value

    def set_arms_position(self):
        self.arm2.setPosition(-1.132)
        self.arm3.setPosition(-1.5)

    def finger_grip(self):
        self.left_finger.setPosition(0)

    def finger_release(self):
        self.left_finger.setPosition(0.025)

    def set_arms_position_on_wall(self):
        self.arm2.setPosition(-0.42)
        self.arm3.setPosition(-1.3)
        self.arm4.setPosition(-1.1)

    def put_box_on_plate2(self):
        # self.arm2.setPosition(0.5)
        # self.arm3.setPosition(1.1)
        # self.arm4.setPosition(1)
        self.arm2.setPosition(0.705)
        target_time: Any = self.getTime() + 0.5
        while self.getTime() < target_time:
            self.step(self.time_step)
        self.arm3.setPosition(0.8)
        target_time: Any = self.getTime() + 0.5
        while self.getTime() < target_time:
            self.step(self.time_step)
        self.arm4.setPosition(1.6)
        target_time: Any = self.getTime() + 1
        while self.getTime() < target_time:
            self.step(self.time_step)

    def PID_step(self, velocity: float = MAX_VELOCITY):

        global last_error, integral

        value = self.get_sensors_value()
        # print(f'{value}')
        error = 0 - value

        # Get P term of the PID.
        p = KP * error

        # Get D term of the PID.
        d = KD * (last_error - error)

        # Update last_error to be used in the next iteration.
        last_error = error

        # Get I term of the PID.
        i = KI * integral
        # Update integral to be used in the next iteration.
        integral += error

        steering = p + d + i

        self.run_motors_steering(steering, velocity)

    def run_motors_steering(self, steering: float, velocity: float = MAX_VELOCITY):

        right_velocity = velocity if steering < 0 else range_conversion(
            0,
            100,
            velocity,
            -velocity,
            steering
        )
        left_velocity = velocity if steering > 0 else range_conversion(
            0,
            -100,
            velocity,
            -velocity,
            steering
        )

        self.set_motors_velocity(
            right_velocity,
            left_velocity,
            right_velocity,
            left_velocity,
        )

    def turn_cw(self, velocity: float):
        self.set_motors_velocity(-velocity, velocity, -velocity, velocity)

    def turn_ccw(self, velocity: float):
        self.set_motors_velocity(velocity, -velocity, velocity, -velocity)

    def PID(self, velocity: float = MAX_VELOCITY):

        stage = 0

        while self.step(self.time_step) != -1:

            index = self.has_box_arrived.index(
                False
            ) if False in self.has_box_arrived else 0

            if len(self.color) == 4 and self.branch != -1 and self.branch != -11:
                if None in Map.values():
                    self.handle_surface_color(velocity)
            else:
                self.handle_color_order()

            if not self.has_box and self.branch == -1:
                if self.wall_sensor.getValue() < 1000:
                    self.take_box(velocity)

            else:
                box_color = self.color[index]
                self.detect_surface(box_color, velocity, index, 'first')

            middle_sensor_value = self.sensors[3].getValue()
            center_sensor_value = self.center_sensor.getValue()

            if 500 < center_sensor_value < 600 and stage == 0 and not middle_sensor_value < 600:

                if None in Map.values() and len(self.color) == 4:

                    if self.intersection == 1:

                        if Map[(1, 0)] is None and self.branch == -1:
                            self.turn_clockwise(velocity)

                        elif Map[(1, 1)] is None and self.branch == 0:
                            while self.step(self.time_step) != -1:
                                self.PID_step(velocity)

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if not 500 < center_sensor_value < 600:
                                    self.branch = 1
                                    break
                        else:
                            self.turn_counter_clockwise(velocity)

                elif self.intersection == 1 and self.branch == -11:

                    while self.step(self.time_step) != -1:
                        self.PID_step(velocity)

                        middle_sensor_value = self.sensors[3].getValue()
                        center_sensor_value = self.center_sensor.getValue()

                        if not 500 < center_sensor_value < 600:
                            self.branch = -1
                            break

                elif self.has_box:
                    coordination = (0, 0)
                    for k, v in Map.items():
                        if v == box_color:
                            coordination = k

                    if coordination[0] == self.intersection:

                        if coordination[1] == 0:
                            self.turn_clockwise(velocity)

                        elif coordination[1] == 1:
                            self.turn_counter_clockwise(velocity)
                    else:
                        while self.step(self.time_step) != -1:
                            self.PID_step(velocity)

                            middle_sensor_value = self.sensors[3].getValue(
                            )
                            center_sensor_value = self.center_sensor.getValue()

                            if not 500 < center_sensor_value < 600:
                                self.intersection = 2
                                break
                else:
                    if self.branch == 0:
                        self.turn_counter_clockwise(velocity)

                    elif self.branch == 1:
                        self.turn_clockwise(velocity)

            elif 500 < center_sensor_value < 600 and middle_sensor_value < 600:

                stage = 2

                if None in Map.values() and len(self.color) == 4:

                    if self.intersection == 1:

                        if Map[(1, 0)] is None and self.branch == -1:
                            while self.step(self.time_step) != -1:
                                self.turn_cw(velocity)

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
                                    self.branch = 0
                                    break

                        elif Map[(1, 1)] is None and self.branch == 0:
                            pass

                        else:
                            while self.step(self.time_step) != -1:
                                self.turn_ccw(velocity)

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
                                    self.intersection = 2
                                    self.branch = -11
                                    break

                    elif self.intersection == 2:

                        if Map[(2, 0)] is None and self.branch == -11:
                            while self.step(self.time_step) != -1:
                                self.turn_cw(velocity)

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
                                    self.branch = 0
                                    break

                        elif Map[(2, 1)] is None and self.branch == 0:
                            while self.step(self.time_step) != -1:
                                self.turn_cw(velocity)

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
                                    self.branch = 1
                                    break

                elif self.has_box:
                    coordination = (0, 0)
                    for k, v in Map.items():
                        if v == box_color:
                            coordination = k

                    if coordination[0] == self.intersection:

                        if coordination[1] == 0:
                            while self.step(self.time_step) != -1:
                                self.turn_cw(velocity)

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
                                    self.branch = 0
                                    break

                        elif coordination[1] == 1:
                            while self.step(self.time_step) != -1:
                                self.turn_ccw(velocity)

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
                                    self.branch = 1
                                    break
                else:
                    if self.intersection == 1:

                        if self.branch == 0:
                            self.turn_counter_clockwise(velocity)
                            self.branch = -1

                        elif self.branch == 1:
                            self.turn_clockwise(velocity)
                            self.branch = -1

                    else:
                        if self.branch == 1:
                            while self.step(self.time_step) != -1:
                                self.turn_cw(velocity)

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
                                    self.intersection = 1
                                    self.branch = -11
                                    break
                        elif self.branch == 0:
                            while self.step(self.time_step) != -1:
                                self.turn_ccw(velocity)

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
                                    self.intersection = 1
                                    self.branch = -11
                                    break

            else:
                if self.has_box is True:
                    self.PID_step(3 * velocity/4)
                else:
                    self.PID_step(velocity)

                if 600 < center_sensor_value:
                    stage = 0

    def take_box(self, velocity: float):

        self.set_motors_velocity(0, 0, 0, 0)

        self.set_arms_position_on_wall()
        target_time: Any = self.getTime() + 3
        while self.getTime() < target_time:
            self.step(self.time_step)

        self.finger_grip()
        target_time: Any = self.getTime() + 2
        while self.getTime() < target_time:
            self.step(self.time_step)

        self.put_box_on_plate2()
        target_time: Any = self.getTime() + 3
        while self.getTime() < target_time:
            self.step(self.time_step)

        # self.finger_release()
        # target_time: Any = self.getTime() + 2
        # while self.getTime() < target_time:
        #     self.step(self.time_step)

        self.has_box = True

        self.turn_cw(velocity)
        target_time: Any = self.getTime() + 2.5
        while self.getTime() < target_time:
            self.step(self.time_step)

    def handle_color_order(self):
        camera_array = self.camera.getImageArray()
        camera_array = np.array(camera_array)

        red = camera_array[0, 0, 0]
        green = camera_array[0, 0, 1]
        blue = camera_array[0, 0, 2]

        if not np.any(camera_array):
            pass
        elif red == green == blue:
            pass
        else:
            if self.color.count('red') == 0 and green == 0 and blue == 0:
                self.color.append("red")

            elif self.color.count('green') == 0 and red == 0 and blue == 0:
                self.color.append("green")

            elif self.color.count('blue') == 0 and green == 0 and red == 0:
                self.color.append("blue")

            elif self.color.count('yellow') == 0 and green != 0 and red != 0 and blue == 0:
                self.color.append("yellow")

    def handle_surface_color(self, velocity: float):
        bottom_camera_array = self.camera.getImageArray()
        bottom_camera_array = np.array(bottom_camera_array)

        red = bottom_camera_array[0, 0, 0]
        green = bottom_camera_array[0, 0, 1]
        blue = bottom_camera_array[0, 0, 2]

        if not np.any(bottom_camera_array):
            pass
        elif red == green == blue:
            pass
        else:
            if 'red' not in Map.values() and green == 0 and blue == 0:
                Map[(self.intersection, self.branch)] = 'red'

            elif 'green' not in Map.values() and red == 0 and blue == 0:
                Map[(self.intersection, self.branch)] = 'green'

            elif 'blue'not in Map.values() and green == 0 and red == 0:
                Map[(self.intersection, self.branch)] = 'blue'

            elif 'yellow'not in Map.values() and green != 0 and red != 0 and blue == 0:
                Map[(self.intersection, self.branch)] = 'yellow'

            self.turn_cw(velocity)
            target_time: Any = self.getTime() + 2.5
            while self.getTime() < target_time:
                self.step(self.time_step)

    def detect_surface(self, box_color: str, velocity: float, index: int, box: str):
        bottom_camera_array = self.camera.getImageArray()
        bottom_camera_array = np.array(bottom_camera_array)

        red = bottom_camera_array[0, 0, 0]
        green = bottom_camera_array[0, 0, 1]
        blue = bottom_camera_array[0, 0, 2]

        if not np.any(bottom_camera_array):
            pass
        elif red == green == blue:
            pass
        else:
            if box_color == 'red' and green == 0 and blue == 0:
                self.drop_box(velocity)

            elif box_color == 'green' and red == 0 and blue == 0:
                self.drop_box(velocity)

            elif box_color == 'blue' and green == 0 and red == 0:
                self.drop_box(velocity)

            elif box_color == 'yellow' and green != 0 and red != 0 and blue == 0:
                self.drop_box(velocity)

            if box == 'first':
                self.has_box = False

            self.has_box_arrived[index] = True

    def drop_box(self, velocity: float):

        self.set_motors_velocity(0, 0, 0, 0)

        self.finger_grip()
        target_time = self.getTime() + 2
        while self.getTime() < target_time:
            self.step(self.time_step)

        self.set_arms_position()
        target_time = self.getTime() + 2
        while self.getTime() < target_time:
            self.step(self.time_step)

        self.finger_release()
        target_time = self.getTime() + 2
        while self.getTime() < target_time:
            self.step(self.time_step)

        self.arm2.setPosition(0)
        self.arm3.setPosition(0)
        self.arm4.setPosition(0)

        target_time = self.getTime() + 3
        while self.getTime() < target_time:
            self.step(self.time_step)

        self.turn_cw(velocity)
        target_time: Any = self.getTime() + 2.5
        while self.getTime() < target_time:
            self.step(self.time_step)

    def turn_clockwise(self, velocity: float):
        while self.step(self.time_step) != -1:
            self.turn_cw(velocity)

            middle_sensor_value = self.sensors[3].getValue()
            center_sensor_value = self.center_sensor.getValue()

            if 500 < center_sensor_value < 600 and middle_sensor_value < 600:
                break

    def turn_counter_clockwise(self, velocity: float):
        while self.step(self.time_step) != -1:
            self.turn_ccw(velocity)

            middle_sensor_value = self.sensors[3].getValue()
            center_sensor_value = self.center_sensor.getValue()

            if 500 < center_sensor_value < 600 and middle_sensor_value < 600:
                break


r = RobotController()
r.PID(10)
