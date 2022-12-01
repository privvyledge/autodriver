Header header

bool first_control_cmd_sent  # Notifies the state machine that a control command was sent to the DBW system
bool first_state_cmd_sent  # Notifies the state machine that a state command was sent to the DBW system

uint64 disabled_feedback_count

uint64 disabled_feedback_thresh  # If state = ENABLE_SENT and DBW reports DISABLED, debounce this many msgs

DbwState state  # the current state of the Dbw system