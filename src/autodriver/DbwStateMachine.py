"""
Similar to DbwStateMachine in Autoware.Auto's vehicle info. Can be used with both ROS1 and ROS2 but isn't necessary for
ROS2 as Autoware.Auto's implementation is in C.

Could Rename to Vehicle State Machine.

Source: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/src/drivers/vehicle_interface/src/dbw_state_machine.cpp
Todo: create message for this.
Todo: put type annotations for Python3
Todo: decide if I should just implement DbwState here instead of importing
"""
from DbwState import DbwState


class DbwStateMachine(object):
    """docstring for ClassName"""

    def __init__(self, dbw_disabled_debounce):
        """
        Brief class for maintaining the DBW state.
        :param dbw_disabled_debounce: If state = ENABLE_SENT and DBW reports DISABLED, debounce this many msgs
        """
        super(DbwStateMachine, self).__init__()
        self.first_control_cmd_sent = False
        self.first_state_cmd_sent = False
        self.disabled_feedback_count = 0
        self.DISABLED_FEEDBACK_THRESH = dbw_disabled_debounce
        self.state = DbwState.DISABLED  # int, but can change depending on if I modify DbwState.py

    def enabled(self):
        """
        Returns true if state is ENABLED, ENABLE_SENT, or ENABLE_REQUESTED with conditions
        :return: bool
        """
        return self.state == DbwState.ENABLED or \
               self.state == DbwState.ENABLE_SENT or \
               (self.state == DbwState.ENABLE_REQUESTED and
                self.first_control_cmd_sent and
                self.first_state_cmd_sent)

    def get_state(self):
        """
        Returns true if state is ENABLED, ENABLE_SENT, or ENABLE_REQUESTED with conditions.
        :return: A DbwState object representing the current state
        """
        return self.state

    def dbw_feedback(self, enabled):
        """
        Notifies the state machine that feedback was received from the DBW system.
        Takes the DBW state from DbwState.ENABLE_SENT to DbwState.ENABLED
        :param enabled: If true, DBW system reports enabled. If false, DBW system reports disabled
        :return: bool.
        """
        if enabled:  # DBW system says enabled
            if self.state == DbwState.ENABLE_SENT:  # and state is ENABLE_SENT
                self.state = DbwState.ENABLED
                self.disabled_feedback_count = 0

        else:  # DBW system says disabled
            if self.state == DbwState.ENABLE_SENT:  # and state is ENABLE_SENT
                self.disabled_feedback_count += 1  # Increase debounce count

                if self.disabled_feedback_count > self.DISABLED_FEEDBACK_THRESH:  # check debounce
                    self.disable_and_reset()

            elif self.state == DbwState.ENABLED:  # and state is ENABLED
                self.disable_and_reset()

    def control_cmd_sent(self):
        """
        Notifies the state machine that a control command was sent to the DBW system.
        Takes the system from DbwState.ENABLE_REQUESTED to DbwState.ENABLE_SENT
        :return:
        """
        if self.state == DbwState.ENABLE_REQUESTED and \
                self.first_control_cmd_sent and \
                self.first_state_cmd_sent:
            # We just sent a state command with enable == True so we can transition to ENABLE_SENT
            self.state = DbwState.ENABLE_SENT

        if self.state == DbwState.ENABLE_REQUESTED:
            self.first_control_cmd_sent = True

    def state_cmd_sent(self):
        """
        Notifies the state machine that a state command was sent to the DBW system
        Takes the system from DbwState.ENABLE_REQUESTED to DbwState.ENABLE_SENT
        :return:
        """
        if self.state == DbwState.ENABLE_REQUESTED and \
                self.first_control_cmd_sent and \
                self.first_state_cmd_sent:
            # We just sent a state command with enable == True so we can transition to ENABLE_SENT
            self.state = DbwState.ENABLE_SENT

        if self.state == DbwState.ENABLE_REQUESTED:
            self.first_state_cmd_sent = True

    def user_request(self, enable):
        """
        The user has requested the DBW system to enable (true) or disable (false)
        Takes the DBW state from DbwState.ENABLED to DbwState.ENABLE_REQUESTED
        :param enable: If true, request enable. If false, request disable
        :return:
        """
        if enable:  # Enable is being requested
            if self.state == DbwState.DISABLED:  # only change states if currently in DISABLED
                self.state = DbwState.ENABLE_REQUESTED

        else:  # Disable is being requested
            self.disable_and_reset()  # disable in any state if user requests it

    def disable_and_reset(self):
        self.state = DbwState.DISABLED
        self.first_state_cmd_sent = False
        self.first_state_cmd_sent = False
        self.disabled_feedback_count = 0


if __name__ == '__main__':
    dbw_s_m = DbwStateMachine(dbw_disabled_debounce=3)
    state = DbwState()
    print(DbwState.DISABLED)
