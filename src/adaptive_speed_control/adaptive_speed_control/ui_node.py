"""Backward compatibility module re-exporting TeleopUINode as UInode."""

from adaptive_speed_control.teleop_ui_node import (
    TeleopUINode as UInode,
    main,
)

__all__ = ['UInode', 'main']

if __name__ == '__main__':
    main()
