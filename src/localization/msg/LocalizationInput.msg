# Exact pre-localizer-voxel input selected for one registration stage.
# The embedded cloud preserves the original sensor timestamp and frame.
uint64 attempt
string stage
string source_topic
builtin_interfaces/Time selected_stamp
sensor_msgs/PointCloud2 cloud
