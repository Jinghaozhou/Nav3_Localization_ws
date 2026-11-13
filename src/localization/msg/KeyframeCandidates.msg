# One immutable short observation and its map->odom hypotheses. No pose truth.
sensor_msgs/PointCloud2 cloud
geometry_msgs/Transform[] map_to_odom
string[] ids
float64[] scores
int32 region_id
string map_sha256
string library_sha256
string profile_sha256
uint64 window_start_ns
float64 encoding_ms
float64 retrieval_ms
bool ready
string reason
