#!/usr/bin/env python3
import sys
import os
import argparse
import csv
import time
import numpy as np
import cv2
import rosbag
import rospy
from cv_bridge import CvBridge, CvBridgeError

# --- CONFIGURATION FOR STREAM DETECTION ---
VIDEO_TOPICS = [
    "/poe_cam1/image_raw/compressed",
    "/poe_cam2/image_raw/compressed",
    "/poe_cam3/image_raw/compressed"
]

MSE_THRESHOLD = 150
MAX_FRAME_GAP = 1.0  # Time gap threshold in seconds to count as a frozen skip
NON_FROZEN_BUFFER = 0.5  # Extra seconds of padding to preserve on healthy segments

OUTPUT_FILENAME = "frozen_intervals.txt"
MSE_CSV_FILENAME = "mse_over_time.csv"
# -----------------------------------------

def calculate_mse(imageA, imageB):
    """Calculate the Mean Squared Error between two images."""
    if imageA.shape != imageB.shape:
        return float('inf')
    err = np.sum((imageA.astype("float") - imageB.astype("float")) ** 2)
    err /= float(imageA.shape[0] * imageA.shape[1] * (imageA.shape[2] if len(imageA.shape) > 2 else 1))
    return err

def consolidate_intervals(intervals):
    """Merges overlapping or adjacent time intervals."""
    if not intervals:
        return []
    sorted_inv = sorted(intervals, key=lambda x: x[0])
    merged = [sorted_inv[0]]
    for current in sorted_inv[1:]:
        prev_start, prev_end = merged[-1]
        curr_start, curr_end = current
        if curr_start <= prev_end:
            merged[-1] = (prev_start, max(prev_end, curr_end))
        else:
            merged.append(current)
    return merged

def calculate_remaining_intervals(bag_start, bag_end, banned_intervals):
    """Calculates the surviving time segments based on the full bag limits."""
    if not banned_intervals:
        return [(bag_start, bag_end)]
    remaining = []
    current_track = bag_start
    for ban_start, ban_end in banned_intervals:
        if ban_start >= bag_end:
            break
        if ban_end <= bag_start:
            continue
        actual_ban_start = max(bag_start, ban_start)
        actual_ban_end = min(bag_end, ban_end)
        if current_track < actual_ban_start:
            remaining.append((current_track, actual_ban_start))
        current_track = max(current_track, actual_ban_end)
    if current_track < bag_end:
        remaining.append((current_track, bag_end))
    return remaining

def analyze_bag(bag_path):
    """Analyzes the video topics sequentially for frozen intervals or long frame skips."""
    bridge = CvBridge()
    
    topic_states = {
        topic: {'last_cv_img': None, 'last_stamp': None, 'freeze_start_time': None, 'is_frozen': False} 
        for topic in VIDEO_TOPICS
    }
    
    frozen_intervals = []
    mse_range = np.array([])
    print(f"\nAnalyzing bag for frozen feeds (Sequential Safe Mode): {bag_path}")
    start_processing_time = time.time()
    
    try:
        with rosbag.Bag(bag_path, 'r') as bag, open(MSE_CSV_FILENAME, mode='w', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(["Timestamp", "Topic", "MSE", "FrameGap"])
            
            info = bag.get_type_and_topic_info()
            total_frames = sum(info.topics[t].message_count for t in VIDEO_TOPICS if t in info.topics)
            
            if total_frames == 0:
                print("Error: None of the specified topics found or they contain 0 frames.")
                return []
                
            print(f"Total video frames to process: {total_frames}")
            current_frame = 0
            current_time = None
            
            for topic, msg, t in bag.read_messages(topics=VIDEO_TOPICS):
                current_frame += 1
                percent = (current_frame / total_frames) * 100
                
                try:
                    if hasattr(msg, 'format'):
                        cv_img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
                    else:
                        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                except CvBridgeError as e:
                    print(f"\nCvBridge Error on topic {topic}: {e}")
                    continue

                state = topic_states[topic]
                current_time = msg.header.stamp 

                if state['last_cv_img'] is None:
                    state['last_cv_img'] = cv_img
                    state['last_stamp'] = current_time
                    continue

                # Gap / Skip Detection
                time_gap = (current_time - state['last_stamp']).to_sec()
                is_skipped = time_gap > MAX_FRAME_GAP

                # Visual Staleness Detection via MSE
                MSE = calculate_mse(state['last_cv_img'], cv_img)
                mse_range = np.append(mse_range, MSE)
                csv_writer.writerow([f"{current_time.to_sec():.6f}", topic, f"{MSE:.4f}", f"{time_gap:.4f}"])
                
                sys.stdout.write(f"\rProcessing: [{current_frame}/{total_frames}] {percent:.1f}% Complete | MSE: {MSE:.2f} | Gap: {time_gap:.2f}s")
                sys.stdout.flush()
                is_static = MSE < MSE_THRESHOLD

                # State machine tracking
                if is_static or is_skipped:
                    if not state['is_frozen']:
                        state['freeze_start_time'] = state['last_stamp'] if is_skipped else current_time
                        state['is_frozen'] = True
                else:
                    if state['is_frozen']:
                        frozen_intervals.append((topic, state['freeze_start_time'].to_sec(), current_time.to_sec()))
                        state['is_frozen'] = False
                        state['freeze_start_time'] = None

                state['last_cv_img'] = cv_img
                state['last_stamp'] = current_time

            for topic, state in topic_states.items():
                if state['is_frozen'] and state['freeze_start_time'] is not None and current_time is not None:
                    frozen_intervals.append((topic, state['freeze_start_time'].to_sec(), current_time.to_sec()))

    except Exception as e:
        print(f"\nError reading bag file during analysis: {e}")
        return []

    end_processing_time = time.time()
    elapsed_time = end_processing_time - start_processing_time
    print() 

    # --- PRINT RESULTS ---
    print("\n" + "="*60)
    print("FROZEN / SKIPPED FEED DETECTION REPORT")
    print(f"Bag Analysis Pass Execution Time: {elapsed_time:.2f} seconds")
    
    if mse_range.size > 0:
        print(f" MSE Metrics: min:{np.min(mse_range):.4f}, max:{np.max(mse_range):.4f}, avg:{np.mean(mse_range):.4f}, std:{np.std(mse_range):.4f}")
        print(f"[✔] Dataset with time gaps saved to: {MSE_CSV_FILENAME}")
    print("="*60)
    
    pure_timestamps = []
    if not frozen_intervals:
        print("Success! No frozen feeds or time skips detected across topics.")
    else:
        print(f"Detected {len(frozen_intervals)} raw event instance(s):\n")
        
        total_raw_frozen_duration = 0.0
        for topic, start, end in frozen_intervals:
            duration = end - start
            total_raw_frozen_duration += duration
            pure_timestamps.append((start, end))
            
            print(f" [!] Topic: {topic}")
            print(f"     Start Timestamp : {start:.4f}")
            print(f"     End Timestamp   : {end:.4f}")
            print(f"     Total Duration  : {duration:.2f} seconds")
            print("-" * 50)
            
        consolidated = consolidate_intervals(pure_timestamps)
        unique_frozen_duration = sum((end - start) for start, end in consolidated)
        
        print("\n" + "="*60)
        print("CUMULATIVE TIME ANALYSIS")
        print("="*60)
        print(f" Sum of all frozen intervals (Raw total) : {total_raw_frozen_duration:.2f} seconds")
        print(f" Unique timeline time frozen (Merged)    : {unique_frozen_duration:.2f} seconds")
        print("="*60)
        
        try:
            with open(OUTPUT_FILENAME, "w") as f:
                for start, end in pure_timestamps:
                    f.write(f"{start:.6f},{end:.6f}\n")
            print(f"[✔] Successfully saved raw intervals to text file: {OUTPUT_FILENAME}")
        except IOError as e:
            print(f"[❌] Error writing to file {OUTPUT_FILENAME}: {e}")

    return pure_timestamps

def main():
    parser = argparse.ArgumentParser(
        description="Detect frozen video streams in a ROS bag and cut those segments out while protecting non-frozen footage boundaries."
    )
    parser.add_argument('-i', '--input', required=True, help="Path to input .bag file")
    parser.add_argument('-o', '--output', required=True, help="Path to save output modified .bag file")

    args = parser.parse_args()

    if not os.path.exists(args.input):
        print(f"Error: The input bag file '{args.input}' does not exist.")
        sys.exit(1)

    # Start overall operation timer
    script_start_time = time.time()

    print(f"Scanning input bag metadata: {args.input}")
    try:
        in_bag = rosbag.Bag(args.input)
        bag_start = in_bag.get_start_time()
        bag_end = in_bag.get_end_time()
    except Exception as e:
        print(f"Error reading bag metadata: {e}")
        sys.exit(1)

    print(f"  Original Bag Range: {bag_start:.4f} to {bag_end:.4f} ({bag_end - bag_start:.2f} seconds)")

    raw_cuts = analyze_bag(args.input)
    banned_sections = consolidate_intervals(raw_cuts)

    # Apply safety padding to preserve healthy footage
    # We shift the cut boundaries inward, effectively expanding the healthy non-frozen window.
    valid_banned_sections = []
    for s, e in banned_sections:
        padded_start = s + NON_FROZEN_BUFFER
        padded_end = e - NON_FROZEN_BUFFER
        
        # If the frozen window was smaller than the total buffer size, it gets completely eliminated
        if padded_start >= padded_end:
            continue
            
        if padded_end <= bag_start or padded_start >= bag_end:
            continue
            
        valid_banned_sections.append((max(bag_start, padded_start), min(bag_end, padded_end)))

    remaining_intervals = calculate_remaining_intervals(bag_start, bag_end, valid_banned_sections)

    print("\n================ TIMELINE EXCISAL ANALYSIS ================")
    print(f"Loaded {len(raw_cuts)} raw events -> Adjusted to preserve extra {NON_FROZEN_BUFFER}s of healthy frames.")
    print(f"Resulted in {len(valid_banned_sections)} distinct cutting blocks.")
    print("\n[The following global intervals will be kept and shifted]:")
    for idx, (s, e) in enumerate(remaining_intervals, 1):
        print(f"  Interval #{idx}: {s:.4f} --> {e:.4f} ({e - s:.2f}s)")
    print("===========================================================\n")

    messages_written = 0
    print("Processing and recalculating timestamps...")
    
    try:
        with rosbag.Bag(args.output, 'w') as outbag:
            accumulated_shift = 0.0
            last_processed_edge = bag_start

            for idx, (start_f, end_f) in enumerate(remaining_intervals, 1):
                gap = start_f - last_processed_edge
                accumulated_shift += gap
                
                print(f"  Writing Interval #{idx}/{len(remaining_intervals)} (Shifted forward by -{accumulated_shift:.2f}s)...")
                
                start_time = rospy.Time.from_sec(start_f)
                end_time = rospy.Time.from_sec(end_f)
                
                for topic, msg, t in in_bag.read_messages(start_time=start_time, end_time=end_time):
                    new_epoch = t.to_sec() - accumulated_shift
                    new_ros_time = rospy.Time.from_sec(new_epoch)
                    
                    if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                        msg_epoch = msg.header.stamp.to_sec() - accumulated_shift
                        msg.header.stamp = rospy.Time.from_sec(msg_epoch)
                    
                    outbag.write(topic, msg, new_ros_time)
                    messages_written += 1
                
                last_processed_edge = end_f

        print("---------------------------------------------------")
        print(f"Success! Output saved to: {args.output}")
        print(f"Total Messages Saved: {messages_written}")
        
        if messages_written > 0:
            out_bag_check = rosbag.Bag(args.output)
            new_duration = out_bag_check.get_end_time() - out_bag_check.get_start_time()
            print(f"New Compressed Bag Duration: {new_duration:.2f} seconds.")
            out_bag_check.close()

    except Exception as e:
        print(f"An error occurred during filtering: {e}")
    finally:
        in_bag.close()

    # Calculate and output total benchmark time
    script_total_time = time.time() - script_start_time
    print("\n" + "░"*60)
    print(f" TOTAL OPERATION COMPUTATION TIME: {script_total_time:.2f} seconds")
    print("░"*60 + "\n")

if __name__ == '__main__':
    main()