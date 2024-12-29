"""color_detection controller."""

# You need first to add camera sensor to the Robot.
# And put the size of the camera sensor to 1x1 (width=1, height=1) from the sensor parameters.
# And it's preferred to set the fieldOfView to a very small value (e.g. 0.00001)

import numpy as np
from controller import Robot, wb, Node


class RobotController(Robot):

    def __init__(self):

        Robot.__init__(Robot)

        self.time_step = int(self.getBasicTimeStep())

        self.camera = self.getDevice("camera")
        self.camera.enable(self.time_step)

        self.step(self.time_step)

    def loop(self):
        while self.step(self.time_step) != -1:
            camera_array = self.camera.getImageArray()

            camera_array = np.array(camera_array)
            if not np.any(camera_array):
                print('black')
            else:
                red = camera_array[0, 0, 0]
                green = camera_array[0, 0, 1]
                blue = camera_array[0, 0, 2]

                if green == 0 and blue == 0:
                    print("red")
                if red == 0 and blue == 0:
                    print("green")
                if green == 0 and red == 0:
                    print("blue")
                if green != 0 and red != 0 and blue == 0:
                    print("yellow")

            # print(f'{red=}')
            # print(f'{green=}')
            # print(f'{blue=}')


r = RobotController()
# print(help(r.camera.getImageArray))
r.loop()
