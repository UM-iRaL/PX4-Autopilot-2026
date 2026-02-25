"""
Non-blocking keyboard input handler for Unix/Linux terminals.

This module provides a context manager for reading keyboard input without blocking
the main program execution.
"""

import os
import sys
import select
import termios
import tty


# Named constants for special keys returned by get_key()
KEY_UP    = 'KEY_UP'
KEY_DOWN  = 'KEY_DOWN'
KEY_LEFT  = 'KEY_LEFT'
KEY_RIGHT = 'KEY_RIGHT'

# Arrow escape sequence suffix → named constant
_ARROW_MAP = {
    ord('A'): KEY_UP,
    ord('B'): KEY_DOWN,
    ord('C'): KEY_RIGHT,
    ord('D'): KEY_LEFT,
}


class KeyboardHandler:
    """Non-blocking keyboard input handler for Unix/Linux.

    Returns single characters for normal keys and named constants
    (KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT) for arrow keys.
    """

    def __init__(self):
        """Initialize keyboard handler"""
        self.settings = None
        if sys.stdin.isatty():
            self.settings = termios.tcgetattr(sys.stdin)

    def __enter__(self):
        """Set terminal to raw mode"""
        if self.settings is not None:
            tty.setraw(sys.stdin.fileno())
        return self

    def __exit__(self, type, value, traceback):
        """Restore terminal settings"""
        if self.settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def get_key(self, timeout=0):
        """Get the most recent key press, draining any buffered repeats.

        When a key is held down, the OS keyboard repeat fills the stdin
        buffer with many copies.  This method drains the entire buffer
        as raw bytes and parses only the *last* key, so switching keys
        feels immediate.

        Arrow keys send a 3-byte escape sequence (ESC [ A/B/C/D).

        Args:
            timeout: Timeout in seconds (0 for non-blocking)

        Returns:
            - Named constant (KEY_UP/DOWN/LEFT/RIGHT) for arrow keys
            - Single character string for normal keys
            - None if no key was pressed
        """
        if not sys.stdin.isatty():
            return None

        if not select.select([sys.stdin], [], [], timeout)[0]:
            return None

        # Read all available bytes at once into a single buffer.
        fd = sys.stdin.fileno()
        buf = os.read(fd, 4096)
        # Keep reading while more is immediately available.
        while select.select([sys.stdin], [], [], 0)[0]:
            buf += os.read(fd, 4096)

        # Parse the *last* key from the buffer.
        # Arrow keys are 3 bytes: ESC(0x1b) [ A/B/C/D
        # Normal keys are 1 byte.
        if len(buf) >= 3:
            tail3 = buf[-3:]
            if tail3[0] == 0x1b and tail3[1] == ord('[') and tail3[2] in _ARROW_MAP:
                return _ARROW_MAP[tail3[2]]

        # Last byte is a normal key (or a lone ESC, etc.)
        return chr(buf[-1])
