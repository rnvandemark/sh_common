#!/bin/bash

# Get a ROS name constant.
# For example, to get the name of the ROS parameter HEARTBEAT_PERIOD_MS, the
# call would be 'get_ros_name.sh params.HEARTBEAT_PERIOD_MS'. The smart home
# workspace must have been sourced already.

if [ 1 -ne $# ]; then echo "Incorrect number of arguments given, expected 1." >&2; exit 1; fi

python3 -c "import sh_common_constants; print(sh_common_constants.$1)"
