"""pid controller."""

from typing import Any
import numpy as np
from controller import Robot

MAX_VELOCITY = 14.81

KUKA_WHEEL_RADIUS = 0.05
KUKA_WIDTH = 0.302
KUKA_LENGTH = 0.56

robot_location = {
    'red': [[0.535, 0.515], [-1.144, -1.164]],
    'green': [[-4.364, -4.384], [-3.853, -3.873]],
    'blue': [[0.066, 0.046], [4.18, 4.15]],
    'yellow': [[4.355, 4.335], [-3.95, -3.97]]
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

        self.has_box: bool = False
        self.has_second_box: bool = False

        self.has_box_arrived = [False, False, False, False]

        self.intersection = 1

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

        self.front_right_sensor: Any = self.getDevice("wheel1sensor")
        self.front_left_sensor: Any = self.getDevice("wheel2sensor")
        self.back_right_sensor: Any = self.getDevice("wheel3sensor")
        self.back_left_sensor: Any = self.getDevice("wheel4sensor")

        self.front_right_sensor.enable(self.time_step)
        self.front_left_sensor.enable(self.time_step)
        self.back_right_sensor.enable(self.time_step)
        self.back_left_sensor.enable(self.time_step)

        self.arm1: Any = self.getDevice("arm1")
        self.arm2: Any = self.getDevice("arm2")
        self.arm3: Any = self.getDevice("arm3")
        self.arm4: Any = self.getDevice("arm4")
        self.arm5: Any = self.getDevice("arm5")
        self.left_finger: Any = self.getDevice("finger::left")

        self.left_finger.setPosition(0.025)
        self.arm1.setPosition(0)
        self.arm2.setPosition(0)
        self.arm3.setPosition(0)
        self.arm4.setPosition(0)
        self.arm5.setPosition(0)

        self.left_finger_sensor: Any = self.getDevice("finger::leftsensor")
        self.left_finger_sensor.enable(self.time_step)

        self.center_sensor: Any = self.getDevice("center sensor")
        self.center_sensor.enable(self.time_step)

        self.camera: Any = self.getDevice("camera")
        self.camera.enable(self.time_step)

        self.wall_sensor: Any = self.getDevice("wall sensor")
        self.wall_sensor.enable(self.time_step)

        self.gps: Any = self.getDevice("gps")
        self.gps.enable(self.time_step)

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
        self.arm2.setPosition(-1.1)
        self.arm3.setPosition(-1.3)
        self.arm4.setPosition(-0.8)

    def finger_grip(self):
        self.left_finger.setPosition(0)

    def put_box_on_plate(self):
        self.arm1.setPosition(0)
        self.arm2.setPosition(0.705)
        self.arm3.setPosition(0.7)
        self.arm4.setPosition(1.7)

    def put_second_box_on_plate(self):
        self.arm1.setPosition(0)
        self.arm2.setPosition(0.9)
        self.arm3.setPosition(0.5)
        self.arm4.setPosition(1.7)

    def finger_release(self):
        self.arm3.setPosition(0.8)
        self.arm4.setPosition(1.6)
        self.left_finger.setPosition(0.025)

    def second_finger_release(self):
        self.arm3.setPosition(0.6)
        self.arm4.setPosition(1.6)
        # self.left_finger.setPosition(0.025)

    def put_box_on_wall(self):
        self.arm2.setPosition(-0.42)
        self.arm3.setPosition(-1.15)
        self.arm4.setPosition(-1.55)

    def put_second_box_on_wall(self):
        self.arm2.setPosition(-0.4)
        self.arm3.setPosition(-1.2)
        self.arm4.setPosition(-1.55)

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
            # print(self.intersection)

            index = self.has_box_arrived.index(
                False
            ) if False in self.has_box_arrived else 0

            if len(self.color) == 4:
                pass
            else:
                self.handle_color_order()

            if len(self.color) >= 2:
                # print(self.color)
                box_color = self.color[index]
                second_box_color = self.color[index + 1]
                # print(second_box_color)

                base_location = self.gps.getValues()

                if not self.has_box and robot_location[box_color][0][0] > base_location[0] > robot_location[box_color][0][1] and robot_location[box_color][1][0] > base_location[1] > robot_location[box_color][1][1]:

                    if box_color == 'red':
                        self.set_motors_velocity(0, 0, 0, 0)

                        self.arm1.setPosition(-1.57)
                        target_time: Any = self.getTime() + 1
                        while self.getTime() < target_time:
                            self.step(self.time_step)

                    self.carry_box(velocity)

                if not self.has_second_box and robot_location[second_box_color][0][0] > base_location[0] > robot_location[second_box_color][0][1] and robot_location[second_box_color][1][0] > base_location[1] > robot_location[second_box_color][1][1]:

                    if second_box_color == 'red':
                        self.set_motors_velocity(0, 0, 0, 0)

                        self.arm1.setPosition(-1.57)
                        target_time: Any = self.getTime() + 1
                        while self.getTime() < target_time:
                            self.step(self.time_step)

                    self.carry_second_box(velocity)

            if self.has_second_box:
                if self.wall_sensor.getValue() < 1000:
                    self.deliver_second_box(velocity, index)

            if self.has_box:
                if self.wall_sensor.getValue() < 1000:
                    self.deliver_box(velocity, index)

            middle_sensor_value: Any = self.sensors[3].getValue()
            center_sensor_value: Any = self.center_sensor.getValue()
            front_side_out_line = all(
                sensor.getValue() < 320 for sensor in self.sensors
            )

            if center_sensor_value < 320 and front_side_out_line and len(self.color) == 4:
                self.turn_cw(velocity)

            if 500 < center_sensor_value < 600 and stage == 0 and not middle_sensor_value < 600:

                if self.color[index] == 'red':

                    if self.intersection == 1:

                        if not self.has_box:
                            self.turn_counter_clockwise(velocity)

                        elif self.color[index + 1] == 'blue':

                            if self.has_second_box:
                                self.turn_counter_clockwise(velocity)
                            else:
                                pass
                        else:
                            self.turn_clockwise(velocity)
                    else:
                        if not self.has_box:
                            self.turn_counter_clockwise(velocity)
                        else:
                            self.turn_clockwise2(velocity)

                elif self.color[index] == 'blue':

                    if not self.has_box:
                        self.turn_clockwise(velocity)

                    elif not self.has_second_box:

                        if self.intersection == 1:
                            while self.step(self.time_step) != -1:
                                self.PID_step(velocity)

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if not 500 < center_sensor_value < 600:
                                    self.intersection = 2
                                    break
                    else:
                        if self.intersection == 1:
                            self.turn_clockwise(velocity)

                        elif self.color[index + 1] == 'red' or self.color[index + 1] == 'yellow':
                            self.turn_clockwise(velocity)
                        else:
                            self.turn_counter_clockwise(velocity)

                elif self.color[index] == 'green':

                    if self.intersection == 1:

                        if not self.has_box:
                            self.turn_counter_clockwise(velocity)

                        elif self.color[index + 1] == 'blue':

                            if not self.has_second_box:
                                pass
                            else:
                                self.turn_counter_clockwise(velocity)
                        else:
                            self.turn_clockwise(velocity)
                    else:
                        if not self.has_box:
                            self.turn_clockwise(velocity)

                        elif self.color[index + 1] == 'red' or self.color[index + 1] == 'yellow':
                            self.turn_clockwise(velocity)

                        else:
                            self.turn_counter_clockwise(velocity)

                elif self.color[index] == 'yellow':

                    if self.intersection == 1:

                        if self.has_box:
                            self.turn_counter_clockwise(velocity)

                    else:
                        if not self.has_box:
                            self.turn_counter_clockwise(velocity)

            elif 500 < center_sensor_value < 600 and middle_sensor_value < 600:

                stage = 2

                if self.color[index] == 'red':

                    if self.intersection == 1:

                        if not self.has_box:
                            while self.step(self.time_step) != -1:
                                self.turn_ccw(velocity)

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
                                    self.intersection = 2
                                    break

                        elif self.color[index + 1] == 'blue':

                            if self.has_second_box:
                                self.turn_counter_clockwise2(velocity)
                            else:
                                pass
                        else:
                            self.turn_clockwise2(velocity)
                    else:
                        if not self.has_box:
                            self.turn_counter_clockwise2(velocity)

                        elif self.color[index + 1] == 'yellow':

                            if not self.has_second_box:

                                self.turn_cw(velocity)
                                target_time: Any = self.getTime() + 2
                                while self.getTime() < target_time:
                                    self.step(self.time_step)
                            else:
                                while self.step(self.time_step) != -1:
                                    self.turn_cw(velocity)

                                    middle_sensor_value = self.sensors[3].getValue(
                                    )
                                    center_sensor_value = self.center_sensor.getValue()

                                    if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
                                        self.intersection = 1
                                        break

                        elif self.color[index + 1] == 'green':

                            if not self.has_second_box:
                                self.turn_counter_clockwise2(velocity)
                            else:
                                while self.step(self.time_step) != -1:
                                    self.turn_ccw(velocity)

                                    middle_sensor_value = self.sensors[3].getValue(
                                    )
                                    center_sensor_value = self.center_sensor.getValue()

                                    if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
                                        self.intersection = 1
                                        break
                        else:
                            while self.step(self.time_step) != -1:
                                self.turn_cw(velocity)

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
                                    self.intersection = 1
                                    break

                elif self.color[index] == 'blue':

                    if not self.has_box:
                        self.turn_clockwise2(velocity)

                    elif self.has_second_box and self.intersection == 1:
                        self.turn_clockwise2(velocity)

                    elif self.color[index + 1] == 'red' or self.color[index + 1] == 'yellow':

                        if not self.has_second_box:

                            if self.intersection == 2:
                                self.turn_counter_clockwise2(velocity)
                        else:
                            while self.step(self.time_step) != -1:
                                self.turn_cw(velocity)

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
                                    self.intersection = 1
                                    break

                    elif self.color[index + 1] == 'green':

                        if not self.has_second_box:

                            if self.intersection == 2:
                                self.turn_clockwise2(velocity)
                        else:
                            while self.step(self.time_step) != -1:
                                self.turn_ccw(velocity)

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
                                    self.intersection = 1
                                    break
                    else:
                        self.turn_clockwise2(velocity)

                elif self.color[index] == 'green':

                    if self.intersection == 1:

                        if not self.has_box:
                            while self.step(self.time_step) != -1:
                                self.turn_ccw(velocity)

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
                                    self.intersection = 2
                                    break

                        elif self.color[index + 1] == 'blue':
                            self.turn_counter_clockwise2(velocity)

                        else:
                            self.turn_clockwise2(velocity)

                    else:
                        if not self.has_box:
                            self.turn_clockwise2(velocity)

                        elif self.color[index + 1] == 'red' or self.color[index + 1] == 'yellow':

                            if not self.has_second_box:
                                self.turn_clockwise2(velocity)
                            else:
                                while self.step(self.time_step) != -1:
                                    self.turn_cw(velocity)

                                    middle_sensor_value = self.sensors[3].getValue(
                                    )
                                    center_sensor_value = self.center_sensor.getValue()

                                    if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
                                        self.intersection = 1
                                        break
                        else:
                            while self.step(self.time_step) != -1:
                                self.turn_ccw(velocity)

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
                                    self.intersection = 1
                                    break

                elif self.color[index] == 'yellow':

                    if self.intersection == 1:

                        if not self.has_box:
                            while self.step(self.time_step) != -1:
                                self.turn_ccw(velocity)

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
                                    self.intersection = 2
                                    break

                        else:
                            self.turn_clockwise2(velocity)

                    else:
                        if not self.has_box:
                            self.turn_counter_clockwise2(velocity)

                        else:
                            while self.step(self.time_step) != -1:
                                self.turn_cw(velocity)

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
                                    self.intersection = 1
                                    break

            else:
                # print('pid')
                if self.has_box is True:
                    self.PID_step(3 * velocity/4)
                else:
                    self.PID_step(velocity)

                if 600 < center_sensor_value:
                    stage = 0

    def turn_counter_clockwise2(self, velocity: float):
        while self.step(self.time_step) != -1:
            self.turn_ccw(velocity)

            middle_sensor_value = self.sensors[3].getValue()
            center_sensor_value = self.center_sensor.getValue()

            if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
                break

    def turn_clockwise2(self, velocity: float):
        while self.step(self.time_step) != -1:
            self.turn_cw(velocity)

            middle_sensor_value = self.sensors[3].getValue()
            center_sensor_value = self.center_sensor.getValue()

            if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
                break

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

    def deliver_box(self, velocity: float, index: int):

        self.set_motors_velocity(0, 0, 0, 0)

        self.finger_grip()
        target_time: Any = self.getTime() + 2
        while self.getTime() < target_time:
            self.step(self.time_step)

        self.put_box_on_wall()
        target_time: Any = self.getTime() + 3
        while self.getTime() < target_time:
            self.step(self.time_step)

        self.finger_release()
        target_time: Any = self.getTime() + 2
        while self.getTime() < target_time:
            self.step(self.time_step)

        self.put_box_on_plate()
        target_time: Any = self.getTime() + 2
        while self.getTime() < target_time:
            self.step(self.time_step)

        self.has_box = False
        self.has_box_arrived[index] = True

        self.turn_cw(velocity)
        target_time: Any = self.getTime() + 2.5
        while self.getTime() < target_time:
            self.step(self.time_step)

    def deliver_second_box(self, velocity: float, index: int):

        self.set_motors_velocity(0, 0, 0, 0)

        self.finger_grip()
        target_time: Any = self.getTime() + 2
        while self.getTime() < target_time:
            self.step(self.time_step)

        self.put_box_on_wall()
        target_time: Any = self.getTime() + 3
        while self.getTime() < target_time:
            self.step(self.time_step)

        self.finger_release()
        target_time: Any = self.getTime() + 2
        while self.getTime() < target_time:
            self.step(self.time_step)

        self.put_box_on_plate()
        target_time: Any = self.getTime() + 2
        while self.getTime() < target_time:
            self.step(self.time_step)

        self.has_second_box = False
        self.has_box_arrived[index + 1] = True

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

    def carry_box(self, velocity: float):

        self.set_motors_velocity(0, 0, 0, 0)

        self.set_arms_position()
        target_time: Any = self.getTime() + 2
        while self.getTime() < target_time:
            self.step(self.time_step)

        self.finger_grip()
        target_time: Any = self.getTime() + 2
        while self.getTime() < target_time:
            self.step(self.time_step)

        self.put_box_on_plate()
        target_time: Any = self.getTime() + 2
        while self.getTime() < target_time:
            self.step(self.time_step)

        self.finger_release()
        target_time: Any = self.getTime() + 2
        while self.getTime() < target_time:
            self.step(self.time_step)

        self.has_box = True

        self.turn_cw(velocity)
        target_time: Any = self.getTime() + 2.5
        while self.getTime() < target_time:
            self.step(self.time_step)

    def carry_second_box(self, velocity: float):

        self.set_motors_velocity(0, 0, 0, 0)

        self.set_arms_position()
        target_time: Any = self.getTime() + 2
        while self.getTime() < target_time:
            self.step(self.time_step)

        self.finger_grip()
        target_time: Any = self.getTime() + 2
        while self.getTime() < target_time:
            self.step(self.time_step)

        self.put_second_box_on_plate()
        target_time: Any = self.getTime() + 2
        while self.getTime() < target_time:
            self.step(self.time_step)

        self.second_finger_release()
        target_time: Any = self.getTime() + 2
        while self.getTime() < target_time:
            self.step(self.time_step)

        self.has_second_box = True

        self.turn_cw(velocity)
        target_time: Any = self.getTime() + 2.5
        while self.getTime() < target_time:
            self.step(self.time_step)


robot = RobotController()
robot.PID(10)

#   Red: turn_ccw
#   Blue: turn_cw
#   Green: turn_ccw then turn_cw
#   Yellow: turn_ccw then turn_ccw
