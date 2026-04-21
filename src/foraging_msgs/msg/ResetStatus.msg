string state        # running | success | failed | error | waiting_for_optitrack
float32 pos_error   # metres from goal (0.0 when not yet computed)
float32 yaw_error   # radians from goal yaw (0.0 when not yet computed)
string error_msg    # populated when state == error or failed
