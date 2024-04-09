import os
import pygame

from .fi_logger import Logger


class JoystickXBOX:
    def __init__(self):
        result = pygame.init()
        Logger().print_trace("pygame.init() result: " + str(result))

        os.environ["SDL_VIDEODRIVER"] = "dummy"  # or maybe 'fbcon'
        result = pygame.display.init()
        # screen = pygame.display.set_mode((1, 1))
        # Logger().print_trace("pygame.display.init() result: " + str(result))

        result = pygame.joystick.init()
        Logger().print_trace("pygame.joystick.init() result: " + str(result))

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.joystick_name = self.joystick.get_name()
        self.joystick_numofballs = self.joystick.get_numballs()
        self.joystick_numofaxes = self.joystick.get_numaxes()
        self.joystick_numofhats = self.joystick.get_numhats()
        self.joystick_numofbuttons = self.joystick.get_numbuttons()

        Logger().print_trace("Joystick self.joystick = \n", self.joystick)

        self.hat_left_right = 0
        self.hat_up_down = 0

        self.button_cross = 0
        self.button_circle = 0
        self.button_square = 0
        self.button_triangle = 0

        self.button_l1 = 0
        self.button_l1_last = 0
        self.button_l2 = 0
        self.button_l2_last = 0
        self.button_r1 = 0
        self.button_r1_last = 0
        self.button_r2 = 0
        self.button_r2_last = 0

        self.button_share = 0
        self.button_share_last = 0
        self.button_option = 0
        self.button_option_last = 0

        self.button_logo = 0
        self.button_axis_left = 0
        self.button_axis_right = 0

        self.axis_left = (0, 0)
        self.axis_right = (0, 0)
        self.axis_l2 = 0
        self.axis_r2 = 0

    def upload(self):
        pygame.event.pump()

        self.hat_left_right = self.joystick.get_hat(0)[0]
        self.hat_up_down = self.joystick.get_hat(0)[1]

        self.button_cross = self.joystick.get_button(0)
        self.button_circle = self.joystick.get_button(1)
        self.button_square = self.joystick.get_button(2)
        self.button_triangle = self.joystick.get_button(3)

        self.button_l1_last = self.button_l1
        self.button_l2_last = self.button_l2
        self.button_r1_last = self.button_r1
        self.button_r2_last = self.button_r2

        self.button_l1 = self.joystick.get_button(4)
        self.button_r1 = self.joystick.get_button(5)

        self.button_share_last = self.button_share
        self.button_share = self.joystick.get_button(6)
        self.button_option_last = self.button_option
        self.button_option = self.joystick.get_button(7)

        self.button_logo = self.joystick.get_button(8)
        self.button_axis_left = self.joystick.get_button(9)
        self.button_axis_right = self.joystick.get_button(10)

        self.axis_left = (self.joystick.get_axis(0), self.joystick.get_axis(1))
        self.axis_right = (self.joystick.get_axis(3), self.joystick.get_axis(4))
        self.axis_l2 = self.joystick.get_axis(2)
        self.axis_r2 = self.joystick.get_axis(5)

        if self.axis_l2 > 0:
            self.button_l2 = 1
        else:
            self.button_l2 = 0

        if self.axis_r2 > 0:
            self.button_r2 = 1
        else:
            self.button_r2 = 0

    def get_joystick_name(self):
        return self.joystick_name

    def get_hat_left_right(self):
        return self.hat_left_right

    def get_hat_up_down(self):
        return self.hat_up_down

    def get_button_cross(self):
        return self.button_cross

    def get_button_circle(self):
        return self.button_circle

    def get_button_square(self):
        return self.button_square

    def get_button_triangle(self):
        return self.button_triangle

    def get_button_l1(self):
        return self.button_l1

    def get_button_l1_last(self):
        return self.button_l1_last

    def get_button_l2(self):
        return self.button_l2

    def get_button_l2_last(self):
        return self.button_l2_last

    def get_button_r1(self):
        return self.button_r1

    def get_button_r1_last(self):
        return self.button_r1_last

    def get_button_r2(self):
        return self.button_r2

    def get_button_r2_last(self):
        return self.button_r2_last

    def get_button_share(self):
        return self.button_share

    def get_button_share_last(self):
        return self.button_share_last

    def get_button_options(self):
        return self.button_option

    def get_button_options_last(self):
        return self.button_option_last

    def get_button_logo(self):
        return self.button_logo

    def get_button_axis_left(self):
        return self.button_axis_left

    def get_button_axis_right(self):
        return self.button_axis_right

    def get_axis_left(self):
        return self.axis_left

    def get_axis_right(self):
        return self.axis_right

    def get_axis_l2(self):
        return self.axis_left

    def get_axis_r2(self):
        return self.axis_right
