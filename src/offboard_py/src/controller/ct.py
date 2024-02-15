from inputs import get_gamepad
import math
import threading

class XboxController(object):
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):

        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self.LeftTrigger = 0
        self.RightTrigger = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LeftThumb = 0
        self.RightThumb = 0
        self.Back = 0
        self.Start = 0
        self.LeftDPad = 0
        self.RightDPad = 0
        self.UpDPad = 0
        self.DownDPad = 0
        self.Joy = []
        self.JoystickInputs()

    def JoystickInputs(self):
        
        while True:
            events = get_gamepad()
            for event in events:
                if event.code == 'ABS_Y':
                    self.LeftJoystickY = round(event.state / XboxController.MAX_JOY_VAL) # normalize between -1 and 1
                    self.Joy[0] = self.LeftJoystickY
                elif event.code == 'ABS_X':
                    self.LeftJoystickX = round(event.state / XboxController.MAX_JOY_VAL) # normalize between -1 and 1
                    self.Joy[1] = self.LeftJoystickX
                elif event.code == 'ABS_RY':
                    self.RightJoystickY = round(event.state / XboxController.MAX_JOY_VAL) # normalize between -1 and 1
                    self.Joy[2] = self.RightJoystickY
                elif event.code == 'ABS_RX':
                    self.RightJoystickX = round(event.state / XboxController.MAX_JOY_VAL) # normalize between -1 and 1
                    self.Joy[3] = self.RightJoystickX
                elif event.code == 'ABS_Z':
                    self.LeftTrigger = round(event.state / XboxController.MAX_TRIG_VAL) # normalize between 0 and 1
                    self.Joy[4] = self.LeftTrigger
                elif event.code == 'ABS_RZ':
                    self.RightTrigger = round(event.state / XboxController.MAX_TRIG_VAL) # normalize between 0 and 1
                    self.Joy[5] = self.RightTrigger
                elif event.code == 'BTN_TL':
                    self.LeftBumper = round(event.state)
                    self.Joy[6] = self.LeftBumper
                elif event.code == 'BTN_TR':
                    self.RightBumper = round(event.state)
                    self.Joy[7] = self.RightBumper
                elif event.code == 'BTN_SOUTH':
                    self.A = round(event.state)
                    self.Joy[8] = self.A
                elif event.code == 'BTN_WEST':
                    self.Y = round(event.state)
                    self.Joy[9] = self.Y
                elif event.code == 'BTN_NORTH':
                    self.X = round(event.state)
                    self.Joy[10] = self.X
                elif event.code == 'BTN_EAST':
                    self.B = round(event.state)
                    self.Joy[11] = self.B
                elif event.code == 'BTN_THUMBL':
                    self.LeftThumb = round(event.state)
                    self.Joy[12] = self.LeftThumb
                elif event.code == 'BTN_THUMBR':
                    self.RightThumb = round(event.state)
                    self.Joy[13] = self.RightThumb
                elif event.code == 'BTN_SELECT':
                    self.Back = round(event.state)
                    self.Joy[14] = self.Back
                elif event.code == 'BTN_START':
                    self.Start = round(event.state)
                    self.Joy[15] = self.Start
                elif event.code == 'ABS_HAT0Y':
                    self.DPadY = round(event.state)
                    self.Joy[16] = self.DPadY
                elif event.code == 'ABS_HAT0X':
                    self.DPadX = round(event.state)
                    self.Joy[17] = self.DPadX

            return self.Joy


if __name__ == '__main__':
    joy = XboxController()
    print(joy.JoystickInputs())