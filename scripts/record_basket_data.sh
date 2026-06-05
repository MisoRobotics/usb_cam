#!/bin/bash

# Configuration
RECORD_DURATION=3         # Duration to record in seconds per iteration
BAG_DIR="/root/workspace/saved_data/bag_data"  # Directory to save bag files

# Array of all robot operations
robot_cmds=(
	"test_behavior move fryer_inspection auto_basket_interface False"
	"test_behavior interface_pickup 3 False"

	# start operations for fryer 1
	"test_behavior move auto_basket_interface canal_interface_slot_1 True"
	"test_behavior hanger_dropoff 1"

	# move robot out of the way
	# take snapshots
	# move robot back
	"echo 1"
	
	# place it in the fryer
	"test_behavior hanger_pickup 1"
	"test_behavior submerge 1 3"
	
	# move robot out of the way
	# take snapshots
	# move robot back
	"echo 1"
	
	# take it out of the fryer
	"test_behavior unsubmerge 1 3 4"
	
	# start operation for fryer 2
	"test_behavior move canal_interface_slot_1 canal_interface_slot_2 True"
	"test_behavior hanger_dropoff 2"
	
	# move robot out of the way
	# take snapshots
	# move robot back
	"echo 2"
	
	"test_behavior hanger_pickup 2"
	"test_behavior submerge 2 3"
	
	# move robot out of the way
	# take snapshots
	# move robot back
	"echo 2"
	
	"test_behavior unsubmerge 2 3 4"

	# start operation for fryer 3
	"test_behavior move canal_interface_slot_2 canal_interface_slot_3 True"
	"test_behavior hanger_dropoff 3"
	
	# move robot out of the way
	# take snapshots
	# move robot back
	"echo 3"
	
	"test_behavior hanger_pickup 3"
	"test_behavior submerge 3 3"
	
	# move robot out of the way
	# take snapshots
	# move robot back
	"echo 3"
	
	"test_behavior unsubmerge 3 3 4"

	# start operation for fryer 4
	"test_behavior move canal_interface_slot_3 canal_interface_slot_4 True"
	"test_behavior hanger_dropoff 4"
	
	# move robot out of the way
	# take snapshots
	# move robot back
	"echo 4"
	
	"test_behavior hanger_pickup 4"
	"test_behavior submerge 4 3"
	
	# move robot out of the way
	# take snapshots
	# move robot back
	"echo 4"
	
	"test_behavior unsubmerge 4 3 4"

	# start operation for fryer 5
	"test_behavior move canal_interface_slot_4 canal_interface_slot_5 True"
	"test_behavior hanger_dropoff 5"
	
	# move robot out of the way
	# take snapshots
	# move robot back
	"echo 5"
	
	"test_behavior hanger_pickup 5"
	"test_behavior submerge 5 3"
	
	# move robot out of the way
	# take snapshots
	# move robot back
	"echo 5"
	
	"test_behavior unsubmerge 5 3 4"

	# start operation for fryer 6
	"test_behavior move canal_interface_slot_5 canal_interface_slot_6 True"
	"test_behavior hanger_dropoff 6"
	
	# move robot out of the way
	# take snapshots
	# move robot back
	"echo 6"
	
	"test_behavior hanger_pickup 6"
	"test_behavior submerge 6 3"
	
	# move robot out of the way
	# take snapshots
	# move robot back
	"echo 6"
	
	"test_behavior unsubmerge 6 3 4"
	
	# place basket back to origin
	"test_behavior move canal_interface_slot_6 auto_basket_interface True"
	"test_behavior interface_dropoff 3"
)

# Create bag directory if it doesn't exist
mkdir -p "$BAG_DIR"

echo "========================================="
echo "Starting Data Collection for ML/Vision"
echo "========================================="

rosservice call /go_to_named_joints "named_joints: 'fryer_inspection'"

# Loop through the indices using the '!' operator
for robot_cmd in "${robot_cmds[@]}"; do

    # Run step 1 for robot
    echo "** Running command **: $robot_cmd"
    eval "$robot_cmd"

	if [[ "${robot_cmd}" =~ ^echo\ ([0-9]+)$ ]]; then
		# allow for a slight lag in capture stream
		# sleep 1.0

		# Extract the integer from the regex capture group
    	EXTRACTED_INT="${BASH_REMATCH[1]}"
		
		echo "Going from fryer $EXTRACTED_INT to fryer inspection"
		eval "test_behavior move canal_interface_slot_$EXTRACTED_INT fryer_inspection False"
		
		# Start rosbag recording
		echo "** Recording rosbag **: ${BAG_DIR}/${TODAY}_camera_data_%05i.bag"
		rosbag record --duration=$RECORD_DURATION /poe_cam1/image_raw/compressed /poe_cam2/image_raw/compressed /poe_cam3/image_raw/compressed -o "${BAG_DIR}/cam_data_.bag"
		
		echo "Going from fryer inspection to fryer $EXTRACTED_INT"
		eval "test_behavior move fryer_inspection canal_interface_slot_$EXTRACTED_INT False"
	fi

done

rosservice call /go_to_named_joints "named_joints: 'fryer_inspection'"

echo "Data capture complete"
