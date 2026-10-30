# Live progress of one coverage execution (the Waypoint action on
# nav_action_follow_path), published latched on /coverage_progress by the
# navigation server and returned by /coverage_progress_status.
# Progress is path distance actually handed to Nav2 FollowPath, not cut area.

uint8 STATUS_IDLE=0
uint8 STATUS_NAVIGATING_TO_START=1
uint8 STATUS_RUNNING=2
uint8 STATUS_CANCELING=3
uint8 STATUS_CANCELED=4
uint8 STATUS_SUCCEEDED=5
uint8 STATUS_FAILED=6

std_msgs/Header header

uint8 status
# idle / navigating_to_start / running / canceling / canceled / succeeded / failed
string status_text
# The action goal's dispatch_id: unique per execution.
string mission_id
# Waypoint goal zone_id; -1 when the caller did not name a zone.
int32 zone_id

# 1-based; 0 while no coverage segment is active (e.g. driving to the start).
int32 current_segment_index
int32 total_segments
int32 completed_segments

# 0.0 .. 1.0
float32 current_segment_progress
# 0.0 .. 1.0, weighted by segment length
float32 overall_progress

float32 current_segment_distance_m
float32 completed_distance_m
float32 total_distance_m
# max(total_distance_m - completed_distance_m, 0)
float32 remaining_distance_m

geometry_msgs/PoseStamped current_pose
geometry_msgs/PoseStamped current_segment_start
geometry_msgs/PoseStamped current_segment_goal

# A checkpoint of this execution is on disk and /resume_coverage can continue it.
bool checkpoint_available
string message
