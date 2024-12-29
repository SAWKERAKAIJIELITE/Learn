"""my_controller controller."""

from controller import Robot


KUKA_WHEEL_RADIUS = 0.05
KUKA_WIDTH = 0.302
KUKA_LENGTH = 0.56


class RobotController(Robot):

    def __init__(self):

        Robot.__init__(self)

        self.time_step = int(self.getBasicTimeStep())

        self.front_right_wheel = self.getDevice("wheel1")
        self.front_left_wheel = self.getDevice("wheel2")
        self.back_right_wheel = self.getDevice("wheel3")
        self.back_left_wheel = self.getDevice("wheel4")

        self.front_right_sensor = self.getDevice("wheel1sensor")
        self.front_left_sensor = self.getDevice("wheel2sensor")
        self.back_right_sensor = self.getDevice("wheel3sensor")
        self.back_left_sensor = self.getDevice("wheel4sensor")

        self.front_right_sensor.enable(self.time_step)
        self.front_left_sensor.enable(self.time_step)
        self.back_right_sensor.enable(self.time_step)
        self.back_left_sensor.enable(self.time_step)

        self.front_right_wheel.setPosition(float("inf"))
        self.front_left_wheel.setPosition(float("inf"))
        self.back_right_wheel.setPosition(float("inf"))
        self.back_left_wheel.setPosition(float("inf"))

        self.front_right_wheel.setVelocity(0)
        self.front_left_wheel.setVelocity(0)
        self.back_right_wheel.setVelocity(0)
        self.back_left_wheel.setVelocity(0)

        self.arm1 = self.getDevice("arm1")
        self.arm2 = self.getDevice("arm2")
        self.arm3 = self.getDevice("arm3")
        self.arm4 = self.getDevice("arm4")
        self.arm5 = self.getDevice("arm5")
        self.finger = self.getDevice("finger::left")

        self.arm1.setPosition(float("inf"))
        self.arm2.setPosition(float("inf"))
        self.arm3.setPosition(float("inf"))
        self.arm4.setPosition(float("inf"))
        self.arm5.setPosition(float("inf"))
        self.finger.setPosition(float("inf"))

        self.arm1.setVelocity(0)
        self.arm2.setVelocity(0)
        self.arm3.setVelocity(0)
        self.arm4.setVelocity(0)
        self.arm5.setVelocity(0)
        self.finger.setVelocity(0)

        self.step(self.time_step)

    def set_motors_velocity(self, wheel1_v: float, wheel2_v: float, wheel3_v: float, wheel4_v: float):
        self.front_right_wheel.setVelocity(wheel1_v)
        self.front_left_wheel.setVelocity(wheel2_v)
        self.back_right_wheel.setVelocity(wheel3_v)
        self.back_left_wheel.setVelocity(wheel4_v)

    def move_forward(self, velocity: float):
        self.set_motors_velocity(velocity, velocity, velocity, velocity)

    def move_backward(self, velocity: float):
        self.set_motors_velocity(-velocity, -velocity, -velocity, -velocity)

    def move_left(self, velocity: float):
        self.set_motors_velocity(velocity, -velocity, -velocity, velocity)

    def move_right(self, velocity: float):
        self.set_motors_velocity(-velocity, velocity, velocity, -velocity)

    def turn_cw(self, velocity: float):
        self.set_motors_velocity(-velocity, velocity, -velocity, velocity)

    def turn_ccw(self, velocity: float):
        self.set_motors_velocity(velocity, -velocity, velocity, -velocity)

    def loop(self):
        while self.step(self.time_step) != -1:
            # self.front_right_wheel.setVelocity(14.8)
            print(f'{self.front_right_sensor.getValue()=}')
            print(f'{self.front_left_sensor.getValue()=}')
            print(f'{self.back_left_sensor.getValue()=}')
            print(f'{self.back_right_sensor.getValue()=}')


r = RobotController()
r.loop()
# print(r.back_left_wheel.getMaxVelocity())
