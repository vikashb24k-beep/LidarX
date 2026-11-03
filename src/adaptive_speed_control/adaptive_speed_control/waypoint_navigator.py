"""Backward compatibility module re-exporting WaypointNavigatorNode as WaypointNavigator."""

from adaptive_speed_control.waypoint_navigator_node import (
    WaypointNavigatorNode as WaypointNavigator,
    main,
)

__all__ = ['WaypointNavigator', 'main']

if __name__ == '__main__':
    main()
