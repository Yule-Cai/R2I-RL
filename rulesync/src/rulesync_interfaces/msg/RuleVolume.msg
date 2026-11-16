# 3D volumetric runtime constraint (parallel to 2D RuleSpec; the 2D
# API is untouched). A rule volume forbids a subset of R^3.
string rule_id
uint64 epoch
# rule_type: 0 = KEEP_OUT_VOLUME (initial type).
uint8 rule_type
uint8 severity
string frame_id
# geometry_type: 0 = AABB, 1 = SPHERE, 2 = CONVEX_VOLUME.
uint8 geometry_type
# AABB: center + dimensions (full extents). SPHERE: center + radius
# in dimensions.x. CONVEX_VOLUME: vertices (convex hull, ordered).
geometry_msgs/Point center
geometry_msgs/Vector3 dimensions
geometry_msgs/Point[] vertices
builtin_interfaces/Time valid_from
builtin_interfaces/Time valid_until
