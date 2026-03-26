#!/usr/bin/env python3

'''
subscribe to pucks registry
subscribe to corner registry
recive number of red, blue and green pucks there are
recive how much exploration vs explotation we will do (how many pucks we want to find before picking them up)

use random walk service to locate pucks -
    RandomWalkServerMessage.srv
    int32 num_pucks_to_find
    ---
    bool done      # find the pucks he need to find

find closer puck, get his color -
use move base to go inside radius of puck -
use service to center tyhe puck
use service to pick it up
bring it to home using move base and position of his corner -
use service to leave it
update registry status of the puck from 0 -> found and confirmed, 1 -> placed home -
update tf of that puck -

ERROR HANDLEING!!!!!!!!!
'''