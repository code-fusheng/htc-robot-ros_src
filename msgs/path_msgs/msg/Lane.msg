int32 id
float32 length
float32 weight
# float32 speed_limit  TODO::每条路在定义的时候就加上速度限制 

bool reverse         # if the road is reversiable

int32[] pre_id
int32[] next_id
nav_msgs/Path path   # position & orientation