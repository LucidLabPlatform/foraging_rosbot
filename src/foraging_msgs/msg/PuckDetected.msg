# A puck is detected by camera and published with its x, y position in the camera frame and distance to the camere frame
std_msgs/Header header     # timestamp + camera frame_id
float32 center_x           # center y position of puck in image
float32 center_y           # center y position of puck in image
float32 distance           # estimated distance from depth image
int32 color                # 0=unknown  1=red  2=green  3=blue