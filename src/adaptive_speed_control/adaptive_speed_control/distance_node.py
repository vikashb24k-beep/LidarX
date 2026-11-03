"""Backward compatibility module re-exporting AdaptiveSpeedControllerNode as DistanceNode."""

from adaptive_speed_control.adaptive_speed_controller_node import (
    AdaptiveSpeedControllerNode as DistanceNode,
    main,
)
from adaptive_speed_control.utils.speed_policy import calculate_adaptive_speed

__all__ = ['DistanceNode', 'calculate_adaptive_speed', 'main']

if __name__ == '__main__':
    main()
