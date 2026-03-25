# ArUco marker detected with depth estimate
std_msgs/Header header     # timestamp + camera frame_id
float32 center_x           # center x position of marker in image
float32 center_y           # center y position of marker in image
float32 distance           # estimated distance from depth image (metres)
int32 marker_id            # ArUco marker ID
