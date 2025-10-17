# Host resource metrics captured by the cockpit health publisher.
#
# Values are expressed as floating point percentages where applicable. Missing
# readings are encoded as NaN in the ROS message.
std_msgs/Header header
float32 cpu_percent
float32 load_avg_1
float32 load_avg_5
float32 load_avg_15
float32 mem_used_percent
float32 mem_total_mb
float32 mem_used_mb
float32 disk_used_percent_root
float32 temp_c
float32 uptime_sec
