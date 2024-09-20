#!/usr/bin/python3
# Python Libraries
from time import sleep

# LCD Libraries
from RPLCD.i2c import CharLCD


class LCDScreenDriver:

    def __init__(self):
        # Initialize LCD Screen
        lcd_i2c_address = 0x27

        self._LCD = CharLCD(
            i2c_expander="PCF8574",
            address=lcd_i2c_address,
            port=1,
            cols=16,
            rows=2,
            dotsize=8,
            charmap="A02",
            auto_linebreaks=True,
            backlight_enabled=True,
        )

    def write_to_screen(self, line1: str = "", line2: str = ""):
        self._LCD.clear()

        # limit line size as to big of a line destroys the view
        spacesAvailable = 16
        line1 = line1[0:spacesAvailable]
        line2 = line2[0:spacesAvailable]

        self._LCD.write_string(line1 + "\r\n")
        self._LCD.write_string(line2)

    def fancy_animation(self, animation_speed=0.4):
        # Calculate the apropriate animation speed
        animation_speed = 1 / animation_speed

        # Custom characters ----------
        char_pacman_left_open = [
            [0x0E, 0x1F, 0x1F, 0x1E, 0x1C, 0x1F, 0x1F,
             0x0E],  # Pac-Man with mouth open
        ]
        char_pacman_right_open = [
            [0x0E, 0x1F, 0x1F, 0x0F, 0x07, 0x1F, 0x1F,
             0x0E],  # Pac-Man with mouth open facing right
        ]
        char_pacman_closed = [
            [0x0E, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F,
             0x0E],  # Pac-Man with mouth closed
        ]
        char_ghost = [
            [0x0E, 0x1F, 0x1F, 0x15, 0x1F, 0x1F, 0x0E, 0x00],  # Ghost
        ]

        # Ghost chaisng Packman ----------
        # Load the custom characters into the LCD
        self._LCD.create_char(0, char_pacman_left_open[0])
        self._LCD.create_char(1, char_pacman_closed[0])
        self._LCD.create_char(2, char_ghost[0])

        # Display sequence
        steps = 20
        for a in range(
                steps
        ):  # Increase range to allow characters to exit screen completely
            self._LCD.clear()

            # Pac-Man position and animation
            if a < 16:  # Continue displaying Pac-Man until he's off-screen
                pac_man_pos = (0, a)
                self._LCD.cursor_pos = pac_man_pos
                if a % 2 == 0:
                    self._LCD.write_string(chr(0))  # Mouth open
                else:
                    self._LCD.write_string(chr(1))  # Mouth closed

            # Ghost position and animation
            if 3 < a < steps + 4:  # Start later and continue until the ghost is off-screen
                ghost_pos = (0, a - 4)  # Maintain spacing
                self._LCD.cursor_pos = ghost_pos
                self._LCD.write_string(chr(2))

            sleep(animation_speed)

        # Packman Chasing Ghost ----------
        # Load the custom characters into the LCD
        self._LCD.create_char(0, char_pacman_right_open[0])
        self._LCD.create_char(1, char_pacman_closed[0])
        self._LCD.create_char(2, char_ghost[0])

        # Display sequence
        steps = 26
        for a in range(
                steps + 4):  # Adjusted range to ensure all characters exit screen
            self._LCD.clear()

            # Ghost position and animation
            ghost_start_pos = steps - 1  # Adjusted for initial off-screen to the right
            ghost_current_pos = ghost_start_pos - a
            if 0 <= ghost_current_pos < 16:
                self._LCD.cursor_pos = (1, ghost_current_pos)
                self._LCD.write_string(chr(2))

            # Pac-Man position and animation
            # Starts 4 positions to the right of the ghost initially
            pac_man_start_pos = ghost_start_pos + 4
            pac_man_current_pos = pac_man_start_pos - a
            if 0 <= pac_man_current_pos < 16:
                self._LCD.cursor_pos = (1, pac_man_current_pos)
                if a % 2 == 0:
                    self._LCD.write_string(chr(0))  # Mouth open
                else:
                    self._LCD.write_string(chr(1))  # Mouth closed

            sleep(animation_speed * 0.3)
