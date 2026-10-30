"""Attach the vendor key_teleop UI to the SSH controlling terminal."""
import os
import termios


def require_terminal():
    try:
        fd = os.open("/dev/tty", os.O_RDWR)
    except OSError as exc:
        raise RuntimeError("keyboard requires an interactive terminal; connect with ssh -t") from exc
    try:
        if not os.isatty(fd):
            raise RuntimeError("keyboard requires a controlling terminal")
    finally:
        os.close(fd)


def main():
    require_terminal()
    fd = os.open("/dev/tty", os.O_RDWR)
    saved = termios.tcgetattr(fd)
    try:
        # ROS launch pipes child stdin. curses must use the controlling TTY.
        for target in (0, 1, 2):
            os.dup2(fd, target)
        from key_teleop.key_teleop import main as vendor_main
        vendor_main()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        os.close(fd)
