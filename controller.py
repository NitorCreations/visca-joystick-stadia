import time
from enum import Enum
import platform
from typing import Iterable, Dict, List, Optional, Union, Any

import pygame

RESET_INTERVAL = 60 # Seconds
LONG_PRESS_TIME = 2 # Seconds
LINUX = platform.system() == 'Linux'

class ButtonFunction(Enum):
    CAM_SELECT_0 = 0
    CAM_SELECT_1 = 1
    CAM_SELECT_2 = 2
    CAM_SELECTS = (CAM_SELECT_0, CAM_SELECT_1, CAM_SELECT_2)

    CONFIRM = 2
    EXIT = 6
    INVERT_TILT = 7

    PRESET_0 = 8
    PRESET_1 = 9
    PRESET_2 = 10
    PRESET_3 = 11
    PRESETS = (PRESET_0, PRESET_1, PRESET_2, PRESET_3)

    FOCUS_NEAR = 16
    FOCUS_FAR = 17

class AxisFunction(Enum):
    PAN = 20
    TILT = 21
    ZOOM = 22
    BRIGHTNESS_UP = 23
    BRIGHTNESS_DOWN = 24


class ControllerInput:
    def __init__(self, function: Union[ButtonFunction, AxisFunction], label: str, xbox_label: Optional[str] = None, **pygame_button_nums: Any):
        self.function = function
        self.ps_label = label
        if xbox_label:
            self.xbox_label = xbox_label
        else:
            self.xbox_label = label

        self.pygame_button_nums: Dict[str, int] = pygame_button_nums

    def get_pygame_button_num(self, controller_type: str) -> Optional[int]:
        os_tag = 'linux' if LINUX else 'win'

        # Order of preference for keys
        key_order = [
            f"{controller_type}_{os_tag}",  # e.g., stadia_linux, ps4_win
            controller_type,              # e.g., stadia, ps4
            os_tag,                       # e.g., linux, win (OS-specific, controller-agnostic)
            'id'                          # generic fallback
        ]

        for key_to_try in key_order:
            if key_to_try in self.pygame_button_nums:
                return self.pygame_button_nums[key_to_try]
        
        return None # No mapping found


inputs = [
    ControllerInput(ButtonFunction.CONFIRM, 'Triangle', 'Y', ps4=2, xbox=3, stadia=3), # PS4 Triangle is ID 2
    ControllerInput(ButtonFunction.CAM_SELECT_0, 'X', 'A', id=0), # ps4=0, xbox=0, stadia=0 implicitly via id
    ControllerInput(ButtonFunction.CAM_SELECT_1, 'O', 'B', id=1), # ps4=1, xbox=1, stadia=1 implicitly via id
    ControllerInput(ButtonFunction.CAM_SELECT_2, 'Triangle', 'Y', ps4=2, xbox=3, stadia=3), # Same as CONFIRM, PS4 Triangle is ID 2
    ControllerInput(AxisFunction.PAN, 'Left Stick', id=0), # L-Stick X Axis
    ControllerInput(AxisFunction.TILT, 'Left Stick', id=1), # L-Stick Y Axis
    ControllerInput(AxisFunction.ZOOM, 'Right Stick', linux=3, win=3, id=3), # R-Stick Y Axis (Stadia Axis 3)
    ControllerInput(AxisFunction.BRIGHTNESS_UP, 'Right Trigger', id=5), # R-Trigger (Stadia Axis 5)
    ControllerInput(AxisFunction.BRIGHTNESS_DOWN, 'Left Trigger', linux=2, win=4, stadia=4), # L-Trigger (Stadia Axis 4)
    ControllerInput(ButtonFunction.FOCUS_NEAR, 'Right Bumper', ps4_linux=4, ps4_win=9, xbox=5, stadia=5), # R1
    ControllerInput(ButtonFunction.FOCUS_FAR, 'Left Bumper', ps4_linux=5, ps4_win=10, xbox=4, stadia=4),   # L1
    ControllerInput(ButtonFunction.PRESET_0, 'D-Pad', ps4=11), # PS4 D-Pad Up
    ControllerInput(ButtonFunction.PRESET_1, 'D-Pad', ps4=12), # PS4 D-Pad Down
    ControllerInput(ButtonFunction.PRESET_2, 'D-Pad', ps4=13), # PS4 D-Pad Left
    ControllerInput(ButtonFunction.PRESET_3, 'D-Pad', ps4=14), # PS4 D-Pad Right
    ControllerInput(ButtonFunction.EXIT, 'Options', ps4_linux=9, ps4_win=6, xbox=7, stadia=7), # Menu/Options
    ControllerInput(ButtonFunction.INVERT_TILT, 'Click L Stick', ps4_linux=10, ps4_win=7, xbox=8, stadia=9), # L3
]

class GameController:
    def __init__(self):
        self.joystick = None
        self._pygame_init()
        self._down_times: Dict[ButtonFunction, float] = {}
        self._long_presses: List[ButtonFunction] = []
        self._short_presses: List[ButtonFunction] = []
        self.last_reset_time = time.time()

        controller_name = self.joystick.get_name()
        if 'Stadia' in controller_name:
            controller_type = 'stadia'
        elif 'Sony' in controller_name or 'PS4' in controller_name or 'Wireless Controller' in controller_name:
            controller_type = 'ps4'
        elif 'Xbox' in controller_name:
            controller_type = 'xbox'
        else:
            # Fallback or raise error for truly unknown controllers
            # For now, let's try a generic approach or raise error as before
            raise ValueError(f"Controller type '{controller_name}' not supported or recognized.")

        self._function_to_pygame: Dict[Union[AxisFunction, ButtonFunction], int] = {}
        for input_obj in inputs:
            pygame_num = input_obj.get_pygame_button_num(controller_type)
            if pygame_num is not None:
                self._function_to_pygame[input_obj.function] = pygame_num
        
        self._pygame_to_button : Dict[int, ButtonFunction] = {
            value: key for key, value in self._function_to_pygame.items() if isinstance(key, ButtonFunction)
        }

    def _pygame_init(self):
        while True:
            try:
                pygame.joystick.quit()
                pygame.display.quit()

                pygame.display.init()
                pygame.joystick.init()
                self.joystick = pygame.joystick.Joystick(0)
            except pygame.error:
                input('No controller found. Please connect one then press enter: ')
            else:
                break

    def wait_for_button_press(self) -> ButtonFunction:
        """Blocks until some button is pressed and then returns what button was pressed"""
        while not pygame.event.peek(eventtype=pygame.JOYBUTTONDOWN):
            time.sleep(0.05)

        event = pygame.event.get(eventtype=pygame.JOYBUTTONDOWN)[0]
        button_id = event.dict['button']
        return self._pygame_to_button[button_id]

    def is_button_pressed(self, button: ButtonFunction) -> bool:
        if button not in self._function_to_pygame:
            return False
        return self.joystick.get_button(self._function_to_pygame[button])

    def get_button_presses(self) -> Iterable[ButtonFunction]:
        presses = []
        for event in pygame.event.get(eventtype=pygame.JOYBUTTONDOWN):
            pygame_id = event.dict['button']
            if pygame_id in self._pygame_to_button:
                presses.append(self._pygame_to_button[pygame_id])
                self._down_times[presses[0]] = time.time()

        return presses

    def _record_long_short_presses(self):
        for event in pygame.event.get(eventtype=pygame.JOYBUTTONUP):
            pygame_id = event.dict['button']
            if pygame_id in self._pygame_to_button:
                button = self._pygame_to_button[pygame_id]
                if button in self._down_times and (time.time() - self._down_times[button]) > LONG_PRESS_TIME:
                    self._long_presses.append(button)
                else:
                    self._short_presses.append(button)

    def get_button_short_presses(self) -> Iterable[ButtonFunction]:
        """get_button_presses returns buttons that have been pressed down,
         where this returns buttons that have been released after a short time.
         """
        self._record_long_short_presses()
        out = self._short_presses
        self._short_presses = []
        return out

    def get_button_long_presses(self) -> Iterable[ButtonFunction]:
        """returns buttons that have been released after being pressed down for a longer time"""
        self._record_long_short_presses()
        out = self._long_presses
        self._long_presses = []
        return out

    def get_axis(self, axis: AxisFunction) -> float:
        if axis not in self._function_to_pygame:
            return 0.0  # Return neutral value if axis is not mapped
        return self.joystick.get_axis(self._function_to_pygame[axis])

    def get_button_name(self, button: ButtonFunction) -> str:
        for input_obj in inputs: # Renamed 'input' to 'input_obj' for clarity
            if input_obj.function == button:
                controller_name = self.joystick.get_name()
                if 'Xbox' in controller_name or 'Stadia' in controller_name:
                    return input_obj.xbox_label
                # elif 'Sony' in controller_name or 'PS4' in controller_name: # ps_label is the default
                #     return input_obj.ps_label
                else: # Default to ps_label for PS4 or other controllers
                    return input_obj.ps_label
        return f"Unknown ({button.name if hasattr(button, 'name') else str(button)})" # Fallback

    def print_mappings(self):
        print('The left stick controls the direction (pan and tilt) of the camera, and the right stick controls zoom.')
        print('Click in the left stick to invert the tilt axis.')
        print()
        print('You can control three cameras one at a time. These buttons select the cameras:')
        for function in [ButtonFunction.CAM_SELECT_0, ButtonFunction.CAM_SELECT_1, ButtonFunction.CAM_SELECT_2]:
            print(f'Camera {function.value + 1}: {self.get_button_name(function)}')

        print()
        print('The triggers control brightness (exposure compensation).')
        print('Focus is auto by default, but you can enter manual focus by pressing both bumpers. '
              'After that, you can adjust focus with the bumpers.')
        print('The D-pad on only the Playstation controller controls presets. '
              'Short presses recall presets and long presses set them.')
        print()
        print('Press options to exit.')


    def refresh_connection(self):
        """In case the controller is disconnected or has some comm problem, we reset the connection periodically."""
        if time.time() > self.last_reset_time + RESET_INTERVAL:
            self._pygame_init()
            self.last_reset_time = time.time()


if __name__ == '__main__':
    controller = GameController()

    while True:
        for event in pygame.event.get(eventtype=pygame.JOYBUTTONDOWN):
            pygame_id = event.dict['button']
            print(pygame_id, controller._pygame_to_button.get(pygame_id, 'UNMAPPED'))

        time.sleep(0.5)
        print([controller.joystick.get_axis(i) for i in range(6)])
