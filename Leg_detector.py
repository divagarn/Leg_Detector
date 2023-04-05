#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from queue import Queue
import time

LASER_TOPIC = '/scan_filtered'
MAX_DEGREE = 60
scan_queue = Queue(maxsize=1)
MIN_LASER_DISTANCE = 0.25
VARIANCE = 0.25
MINIMUM_REPEATED_VALUES = 0
SAME_VALUE_COUNT_DIFFERENCE = 20


def detect_leg(scan_data):
    start_idx = 0
    end_idx = 0
    no_of_leg_found = 0
    prev_val = None
    results = []
    scan_data = np.array(scan_data)
    scan_data[np.isinf(scan_data)] = 0
    scan_data[scan_data <= MIN_LASER_DISTANCE] = MIN_LASER_DISTANCE
    pre_max_val = None
    long_distance = np.max(scan_data)
    for i in range(len(scan_data)):
        max_val = np.sort(scan_data)[-1 - i]
        if pre_max_val is None:
            pre_max_val = max_val
            scan_data[(scan_data >= max_val - VARIANCE) & (scan_data <= max_val + VARIANCE)] = max_val
        else:
            if max_val != pre_max_val:
                scan_data[(scan_data >= max_val - VARIANCE) & (scan_data <= max_val + VARIANCE)] = max_val

    for i in range(len(scan_data) - 1):
        if scan_data[i] != scan_data[i + 1]:
            if prev_val is not None and prev_val == scan_data[i]:
                count = end_idx - start_idx
                if count >= MINIMUM_REPEATED_VALUES:
                    results.append((prev_val, start_idx, end_idx, count))
            prev_val = None
            start_idx = i + 1
        else:
            if prev_val is None:
                prev_val = scan_data[i]
            end_idx = i + 1
    data_analysis = []
    visited_values = []
    # print(scan_data)
    for i in range(len(results)):
        value_match = 0
        leg_found = 0
        for j in range(i + 1, len(results)):
            if results[i][0] != MIN_LASER_DISTANCE and results[i][0] != long_distance:
                if results[i][0] not in visited_values:
                    visited_values.append(results[i][0])
                    if results[i][0] == results[j][0]:
                        count_diff = abs(results[i][3] - results[j][3])
                        value_match += 1
                        if count_diff <= SAME_VALUE_COUNT_DIFFERENCE:
                            leg_found += 1
        data_analysis.append((results[i][0], value_match, leg_found))

    for i in data_analysis:
        if i[1] != 0:
            if i[1] == i[2]:
                no_of_leg_found += 1

    return no_of_leg_found


def laser_scan_callback(scan):
    if scan_queue.full():
        scan_queue.queue.clear()
        scan_queue.put(scan.ranges)
    else:
        scan_queue.put(scan.ranges)


class LegDetector:
    def __init__(self):
        rospy.init_node('leg_detector')
        rospy.Subscriber(LASER_TOPIC, LaserScan, laser_scan_callback)

    def run(self):
        while not rospy.is_shutdown():
            if scan_queue.full():
                scan_data = scan_queue.get()
                back_scan_range = np.append(scan_data[(720 - int(MAX_DEGREE / 2) * 2):],
                                            scan_data[:(0 + int(MAX_DEGREE / 2) * 2)])
                right_scan_range = scan_data[180 - int(MAX_DEGREE / 2) * 2:180 + int(MAX_DEGREE / 2) * 2]
                front_scan_range = scan_data[360 - int(MAX_DEGREE / 2) * 2:360 + int(MAX_DEGREE / 2) * 2]
                left_scan_range = scan_data[540 - int(MAX_DEGREE / 2) * 2:540 + int(MAX_DEGREE / 2) * 2]

                front_leg_detected = detect_leg(front_scan_range)
                back_leg_detected = detect_leg(back_scan_range)
                left_leg_detected = detect_leg(left_scan_range)
                right_leg_detected = detect_leg(right_scan_range)

                total_leg_detected = front_leg_detected + back_leg_detected + left_leg_detected + right_leg_detected
                print("Front Leg Detected = ", front_leg_detected)
                print("Back Leg Detected = ", back_leg_detected)
                print("Left Leg Detected = ", left_leg_detected)
                print("Right Leg Detected = ", right_leg_detected)
                print("Total_leg_detected = ", total_leg_detected)

            time.sleep(2)


if __name__ == '__main__':
    leg_detector_obj = LegDetector()
    leg_detector_obj.run()
