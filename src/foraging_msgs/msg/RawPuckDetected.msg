# RGB camera detect puck using mask
std_msgs/Header header     # timestamp + camera frame_id
float32 center_x           # center x position of puck in image (full-image space)
float32 center_y           # center y position of puck in image (full-image space)
int32   color              # 0=unknown  1=red  2=green  3=blue
int32[] contour_x          # contour x-coordinates (cropped-image space)
int32[] contour_y          # contour y-coordinates (cropped-image space)
