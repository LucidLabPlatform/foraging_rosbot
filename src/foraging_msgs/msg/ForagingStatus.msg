string state        # running | completed | error
int32 mode          # 0=explore, 1=mixed, 2=exploit (-1 when not running)
float32 elapsed_s   # seconds since trial start
int32 pucks_placed  # number of pucks placed (populated on completed)
float32 duration_s  # total trial duration (populated on completed)
string error_msg    # populated when state == error
