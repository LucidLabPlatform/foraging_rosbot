#!/usr/bin/env python3

'''
recives numb of pucks to find
subscribe to puck registry
cound how many pucks we found
use move base to move slaowly arround the map
    perform small steps at a time
    move arround with random pattern
    chose next target pose, from the current one with a small shift in cartesian position and small shift in orientation
once num_pucks_to_find is reached return done True
'''