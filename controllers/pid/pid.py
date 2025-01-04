"""pid controller."""

from typing import Any
import numpy as np
from controller import Robot

MAX_VELOCITY = 14.81

KUKA_WHEEL_RADIUS = 0.05
KUKA_WIDTH = 0.302
KUKA_LENGTH = 0.56

boxes_location = {
    'red': [[0.535, 0.515], [-1.144, -1.164]],
    'green': [[-4.3644, -4.3844], [-3.8533, -3.8733]],
    'blue': [[0.0661, 0.0461], [4.21, 4.18]],
    'yellow': [[4.3557, 4.33573], [-3.95, -3.97]]
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
    """
    This function is responsible for mapping ranges
    examples:
    the mapping of the value 50 from range 0 -> 200 to range -50 -> 50 will be -25
    """
    ration = abs((value - s_start) / (s_end - s_start))
    if d_start < d_end:
        return d_start + abs(d_end - d_start) * ration

    return d_start - abs(d_end - d_start) * ration


class RobotController(Robot):

    def __init__(self):

        Robot.__init__(self)

        self.has_box: bool = False

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

        # self.left_finger.setVelocity(0)
        # self.arm1.setVelocity(0)
        # self.arm2.setVelocity(0)
        # self.arm3.setVelocity(0)
        # self.arm4.setVelocity(0)
        # self.arm5.setVelocity(0)

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

    def finger_release(self):
        # self.arm3.setPosition(0.75)
        # self.arm4.setPosition(1.65)
        self.arm3.setPosition(0.8)
        self.arm4.setPosition(1.6)
        self.left_finger.setPosition(0.025)

    def put_box_on_wall(self):
        self.arm2.setPosition(-0.42)
        self.arm3.setPosition(-1.15)
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

            if len(self.color) == 4:
                pass
            else:
                self.handle_color_order()

            if len(self.color) >= 1:
                try:
                    index = self.has_box_arrived.index(False)
                except ValueError:
                    index = 0
                # print(self.color)
                box_color = self.color[index]

                base_location = self.gps.getValues()

                if not self.has_box and boxes_location[box_color][0][0] > base_location[0] > boxes_location[box_color][0][1] and boxes_location[box_color][1][0] > base_location[1] > boxes_location[box_color][1][1]:

                    if box_color == 'red':
                        self.set_motors_velocity(0, 0, 0, 0)

                        self.arm1.setPosition(-1.57)
                        target_time: Any = self.getTime() + 1
                        while self.getTime() < target_time:
                            self.step(self.time_step)
                        
                        self.carry_box(velocity)

                    else:
                        self.carry_box(velocity)

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

            elif 500 < center_sensor_value < 600 and stage == 0 and not middle_sensor_value < 600:

                index = self.has_box_arrived.index(False)

                if self.color[index] == 'red':

                    if self.intersection == 1:

                        if not self.has_box:
                            while self.step(self.time_step) != -1:
                                self.turn_ccw(velocity)
                                print('intersection 1 phase 1 no cube')

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if 500 < center_sensor_value < 600 and middle_sensor_value < 600:
                                    break

                    else:
                        if not self.has_box:
                            while self.step(self.time_step) != -1:
                                self.turn_ccw(velocity)
                                print('intersection 2 phase 1 no cube')

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if 500 < center_sensor_value < 600 and middle_sensor_value < 600:
                                    break

                elif self.color[index] == 'blue':

                    if not self.has_box:
                        while self.step(self.time_step) != -1:
                            self.turn_cw(velocity)

                            middle_sensor_value = self.sensors[3].getValue()
                            center_sensor_value = self.center_sensor.getValue()

                            if 500 < center_sensor_value < 600 and middle_sensor_value < 600:
                                break

                    else:
                        while self.step(self.time_step) != -1:
                            self.turn_ccw(velocity)

                            middle_sensor_value = self.sensors[3].getValue()
                            center_sensor_value = self.center_sensor.getValue()

                            if 500 < center_sensor_value < 600 and middle_sensor_value < 600:
                                break

                elif self.color[index] == 'green':

                    if self.intersection == 1:

                        if not self.has_box:
                            while self.step(self.time_step) != -1:
                                print('intersection 1 phase 1 no cube')
                                self.turn_ccw(velocity)

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if 500 < center_sensor_value < 600 and middle_sensor_value < 600:
                                    break

                        else:
                            while self.step(self.time_step) != -1:
                                self.turn_cw(velocity)
                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if 500 < center_sensor_value < 600 and middle_sensor_value < 600:
                                    break

                    else:
                        if not self.has_box:
                            while self.step(self.time_step) != -1:
                                print('intersection 2 phase 1 no cube')
                                self.turn_cw(velocity)

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if 500 < center_sensor_value < 600 and middle_sensor_value < 600:
                                    break

                        else:
                            while self.step(self.time_step) != -1:
                                print('intersection 2 phase 1 with cube')
                                self.turn_ccw(velocity)

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if 500 < center_sensor_value < 600 and middle_sensor_value < 600:
                                    break

                elif self.color[index] == 'yellow':

                    if self.intersection == 1:

                        if not self.has_box:
                            while self.step(self.time_step) != -1:
                                self.turn_ccw(velocity)
                                print('intersection 1 phase 1 no cube')

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if 500 < center_sensor_value < 600 and middle_sensor_value < 600:
                                    break

                        # else:
                        #     while self.step(self.time_step) != -1:
                        #         self.turn_cw(velocity)

                        #         middle_sensor_value = self.sensors[3].getValue()
                        #         center_sensor_value = self.center_sensor.getValue()

                        #         if 500 < center_sensor_value < 600 and middle_sensor_value < 600:
                        #             break

                    else:
                        if not self.has_box:
                            while self.step(self.time_step) != -1:
                                self.turn_ccw(velocity)
                                print('intersection 2 phase 1 no cube')

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if 500 < center_sensor_value < 600 and middle_sensor_value < 600:
                                    break

                        # else:
                        #     while self.step(self.time_step) != -1:
                        #         self.turn_cw(velocity)

                        #         middle_sensor_value = self.sensors[3].getValue()
                        #         center_sensor_value = self.center_sensor.getValue()

                        #         if 500 < center_sensor_value < 600 and middle_sensor_value < 600:
                        #             break

            elif 500 < center_sensor_value < 600 and middle_sensor_value < 600:

                stage = 2

                index = self.has_box_arrived.index(False)

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

                        else:
                            while self.step(self.time_step) != -1:
                                self.turn_cw(velocity)

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
                                    break

                    else:
                        if not self.has_box:
                            while self.step(self.time_step) != -1:
                                self.turn_ccw(velocity)

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
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
                        while self.step(self.time_step) != -1:
                            self.turn_cw(velocity)

                            middle_sensor_value = self.sensors[3].getValue()
                            center_sensor_value = self.center_sensor.getValue()

                            if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
                                break

                    else:
                        while self.step(self.time_step) != -1:
                            self.turn_ccw(velocity)

                            middle_sensor_value = self.sensors[3].getValue()
                            center_sensor_value = self.center_sensor.getValue()

                            if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
                                break

                elif self.color[index] == 'green':

                    if self.intersection == 1:

                        if not self.has_box:
                            while self.step(self.time_step) != -1:
                                print('intersection 1 phase 2 no cube')
                                self.turn_ccw(velocity)

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
                                    self.intersection = 2
                                    break

                        else:
                            while self.step(self.time_step) != -1:
                                self.turn_cw(velocity)
                                print('intersection 1 phase 2 with cube')

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
                                    break

                    else:
                        if not self.has_box:
                            while self.step(self.time_step) != -1:
                                print('intersection 2 phase 2 no cube')
                                self.turn_cw(velocity)

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
                                    break

                        else:
                            while self.step(self.time_step) != -1:
                                print('intersection 2 phase 2 with cube')
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
                                # print('intersection 1 phase 2 no cube')

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
                                    self.intersection = 2
                                    break

                        else:
                            while self.step(self.time_step) != -1:
                                self.turn_cw(velocity)
                                # print('intersection 1 phase 2 with cube')

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
                                    break

                    else:
                        if not self.has_box:
                            while self.step(self.time_step) != -1:
                                self.turn_ccw(velocity)
                                # print('intersection 2 phase 2 no cube')

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
                                    break

                        else:
                            while self.step(self.time_step) != -1:
                                self.turn_cw(velocity)
                                # print('intersection 2 phase 2 with cube')

                                middle_sensor_value = self.sensors[3].getValue(
                                )
                                center_sensor_value = self.center_sensor.getValue()

                                if not (500 < center_sensor_value < 600 and middle_sensor_value < 600):
                                    self.intersection = 1
                                    break

            else:
                # print(f'{self.intersection=}')
                if self.has_box is True:
                    # if self.color[index] == 'yellow' and self.color[index] == 'green':
                    #     self.PID_step(velocity/4)
                    self.PID_step(velocity/2)
                else:
                    self.PID_step(velocity)

                if 600 < center_sensor_value:
                    stage = 0

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

    def handle_color_order(self):
        camera_array = self.camera.getImageArray()
        camera_array = np.array(camera_array)

        if not np.any(camera_array):
            pass
        else:
            red = camera_array[0, 0, 0]
            green = camera_array[0, 0, 1]
            blue = camera_array[0, 0, 2]

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


r = RobotController()
r.PID(10)

# Easy Way:
#   Red: turn_ccw
#   Blue: turn_cw
#   Green: turn_ccw then turn_cw
#   Yellow: turn_ccw then turn_ccw

# First Box:
#   Red: turn_cw
#   Blue: turn_ccw
#   Green: turn_ccw
