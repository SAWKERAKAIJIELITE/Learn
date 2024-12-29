"""pid controller."""

import numpy as np
from controller import Robot

MAX_VELOCITY = 14.81
KUKA_WHEEL_RADIUS = 0.05
KUKA_WIDTH = 0.302
KUKA_LENGTH = 0.56
red_range = [[0.127, 0.107], [-0.740, -0.750]]
green_range = [[-4.29, -4.312], [-3.8, -3.825]]
yellow_range = [[4.254, 4.234], [-3.877, -3.897]]
blue_range = [[0.0661, 0.0461], [4.33, 4.31]]
boxes_location = {
    'red': [[0.127, 0.107], [-0.740, -0.750]],
    'green': [[-4.3644, -4.3844], [-3.8533, -3.8733]],
    'blue': [[0.0661, 0.0461], [4.21, 4.18]],
    'yellow': [[4.3557, 4.33573], [-3.9481, -3.9681]]
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

        self.has_box = False

        self.time_step = int(self.getBasicTimeStep())

        self.front_right_wheel = self.getDevice("wheel1")
        self.front_left_wheel = self.getDevice("wheel2")
        self.back_right_wheel = self.getDevice("wheel3")
        self.back_left_wheel = self.getDevice("wheel4")

        self.front_right_wheel.setPosition(float("inf"))
        self.front_left_wheel.setPosition(float("inf"))
        self.back_right_wheel.setPosition(float("inf"))
        self.back_left_wheel.setPosition(float("inf"))

        self.front_right_wheel.setVelocity(0)
        self.front_left_wheel.setVelocity(0)
        self.back_right_wheel.setVelocity(0)
        self.back_left_wheel.setVelocity(0)

        self.front_right_sensor = self.getDevice("wheel1sensor")
        self.front_left_sensor = self.getDevice("wheel2sensor")
        self.back_right_sensor = self.getDevice("wheel3sensor")
        self.back_left_sensor = self.getDevice("wheel4sensor")

        self.front_right_sensor.enable(self.time_step)
        self.front_left_sensor.enable(self.time_step)
        self.back_right_sensor.enable(self.time_step)
        self.back_left_sensor.enable(self.time_step)

        self.arm1 = self.getDevice("arm1")
        self.arm2 = self.getDevice("arm2")
        self.arm3 = self.getDevice("arm3")
        self.arm4 = self.getDevice("arm4")
        self.arm5 = self.getDevice("arm5")
        self.left_finger = self.getDevice("finger::left")

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

        self.left_finger_sensor = self.getDevice("finger::leftsensor")
        self.left_finger_sensor.enable(self.time_step)

        self.center_sensor = self.getDevice("center sensor")
        self.center_sensor.enable(self.time_step)

        self.camera = self.getDevice("camera")
        self.camera.enable(self.time_step)

        self.wall_sensor = self.getDevice("wall sensor")
        self.wall_sensor.enable(self.time_step)

        self.gps = self.getDevice("gps")
        self.gps.enable(self.time_step)

        self.sensors = list(map(lambda v: self.getDevice(f"lfs{v}"), range(8)))

        self.weights = [-1000, -1000, -1000, -1000, 1000, 1000, 1000, 1000]

        for sensor in self.sensors:
            sensor.enable(self.time_step)

        self.step(self.time_step)

        self.color = []

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

    def put_box_on_plate(self):
        self.arm2.setPosition(1.13)
        self.arm3.setPosition(0.1)
        self.arm4.setPosition(1.5)

    def finger_release(self):
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
        """
        The function that is responsible for the line fallowing functionality (with looping).
        """
        stage = 0
        while self.step(self.time_step) != -1:
            near_wall = self.wall_sensor.getValue()
            print(f'{near_wall=}')

            if len(self.color) == 4:
                pass
            else:
                self.handle_color_order()

            if len(self.color) >= 1:
                self.carry_box(self.color[0], velocity)

            if near_wall < 1000 and self.has_box is True:
                self.set_motors_velocity(0, 0, 0, 0)

                self.finger_grip()
                target_time = self.getTime() + 2
                while self.getTime() < target_time:
                    self.step(self.time_step)

                self.put_box_on_wall()
                target_time = self.getTime() + 3
                while self.getTime() < target_time:
                    self.step(self.time_step)

                self.finger_release()
                target_time = self.getTime() + 2
                while self.getTime() < target_time:
                    self.step(self.time_step)

                self.put_box_on_plate()
                target_time = self.getTime() + 2
                while self.getTime() < target_time:
                    self.step(self.time_step)

                self.has_box = False

                self.turn_cw(velocity)
                target_time = self.getTime() + 2.5
                while self.getTime() < target_time:
                    self.step(self.time_step)

                # break

            middle_sensor_value = self.sensors[3].getValue()
            center_sensor_value = self.center_sensor.getValue()
            front_side_out_line = all(
                sensor.getValue() < 320 for sensor in self.sensors
            )

            # print(f'{center_sensor_value=}')
            # print(f'{front_side_out_line=}')

            if center_sensor_value < 320 and front_side_out_line and len(self.color) == 4:
                self.turn_cw(velocity)

            elif 500 < center_sensor_value < 600 and stage == 0 and not middle_sensor_value < 600:
                # stage = 1
                self.turn_ccw(velocity)

            elif 500 < center_sensor_value < 600 and middle_sensor_value < 600:
                stage = 2
                self.turn_ccw(velocity)

            else:
                self.PID_step(velocity)
                if 600 < center_sensor_value:
                    stage = 0

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

            if self.color.count('green') == 0 and red == 0 and blue == 0:
                self.color.append("green")

            if self.color.count('blue') == 0 and green == 0 and red == 0:
                self.color.append("blue")

            if self.color.count('yellow') == 0 and green != 0 and red != 0 and blue == 0:
                self.color.append("yellow")

    def carry_box(self, box_color: str, velocity: float):
        base_location = self.gps.getValues()
        # print(f'{base_location=}')

        if boxes_location[box_color][0][0] > base_location[0] > boxes_location[box_color][0][1] and boxes_location[box_color][1][0] > base_location[1] > boxes_location[box_color][1][1] and not self.has_box:
            self.set_motors_velocity(0, 0, 0, 0)
            self.set_arms_position()
            target_time = self.getTime() + 2
            while self.getTime() < target_time:
                self.step(self.time_step)

            self.finger_grip()
            target_time = self.getTime() + 2
            while self.getTime() < target_time:
                self.step(self.time_step)

            self.put_box_on_plate()
            target_time = self.getTime() + 2
            while self.getTime() < target_time:
                self.step(self.time_step)

            self.finger_release()
            target_time = self.getTime() + 2
            while self.getTime() < target_time:
                self.step(self.time_step)

                self.has_box = True

            self.turn_cw(velocity)
            target_time = self.getTime() + 2.5
            while self.getTime() < target_time:
                self.step(self.time_step)


r = RobotController()
r.PID(10)
