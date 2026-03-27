# RGB camera detect puck using mask
std_msgs/Header header     # timestamp + camera frame_id
float32 center_x           # center x position of puck in image (full-image space)
float32 center_y           # center y position of puck in image (full-image space)
int32   bbox_x             # left edge of bounding box (full-image space)
int32   bbox_y             # top edge of bounding box (full-image space)
int32   bbox_w             # width of bounding box
int32   bbox_h             # height of bounding box
int32   color              # 0=unknown  1=red  2=green  3=blue