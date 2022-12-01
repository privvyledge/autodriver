"""
Source: lines 41-47 in
https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/src/drivers/vehicle_interface/include/vehicle_interface/dbw_state_machine.hpp
"""
# todo: create message for this
# todo: might rename to vehicle state or implement vehicle state separately (e.g as a ROS node)


class DbwState(object):
    """docstring for ClassName"""
    # DbwState
    DISABLED = 0  # False
    ENABLE_REQUESTED = 1
    ENABLE_SENT = 2
    ENABLED = 3  # True
