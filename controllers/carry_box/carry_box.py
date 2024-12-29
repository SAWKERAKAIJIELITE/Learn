"""carry_box controller."""

from controller import Robot


class RobotController(Robot):

    def __init__(self):

        Robot.__init__(self)

        self.time_step = int(self.getBasicTimeStep())

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

        self.wall_sensor = self.getDevice("wall sensor")
        self.wall_sensor.enable(self.time_step)

        self.gps = self.getDevice("gps")
        self.gps.enable(self.time_step)

        self.step(self.time_step)

        self.color = []

    def carry_box(self):
        while self.step(self.time_step) != -1:
            print(f'{self.gps.getValues()=}')

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


r = RobotController()
# r.carry_box()

# Phase_1
# r.set_arms_position()
# target_time = r.getTime() + 2
# while r.getTime() < target_time:
#     r.step(r.time_step)

# r.finger_grip()
# target_time = r.getTime() + 2
# while r.getTime() < target_time:
#     r.step(r.time_step)

# r.put_box_on_plate()
# target_time = r.getTime() + 2
# while r.getTime() < target_time:
#     r.step(r.time_step)

# r.finger_release()
# target_time = r.getTime() + 2
# while r.getTime() < target_time:
#     r.step(r.time_step)

# Phase_2
# r.finger_grip()
# target_time = r.getTime() + 2
# while r.getTime() < target_time:
#     r.step(r.time_step)

# r.put_box_on_wall()
# target_time = r.getTime() + 3
# while r.getTime() < target_time:
#     r.step(r.time_step)

# r.finger_release()
# target_time = r.getTime() + 2
# while r.getTime() < target_time:
#     r.step(r.time_step)

# r.put_box_on_plate()
# target_time = r.getTime() + 2
# while r.getTime() < target_time:
#     r.step(r.time_step)

# [-4.22458, -3.84518, 0.0926801, 2.5] [-4.63172, -4.1307, 0.0124917, -0.785395]
# [0.0448424, -0.786322, 0.10496, 1.57] [0.0453488, -1.28, 0.012492]
# [4.20231, -3.88635, 0.108322, [0.00387677, -0.00289621, 0.6]] [4.60814, -4.16304, 0.0124917, 1.0472]
# [0.0562821, 4.3914, 0.11234, 1.57] [0.05, 4.89, 0.0124917]
# [4.65304, 0.27771, 0.101876, 1.57]
