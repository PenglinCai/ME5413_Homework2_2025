#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import os
import cv2
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, BoundingBox2D  # Ensure the vision_msgs package is installed
from cv_bridge import CvBridge

# File path settings (please modify according to your actual situation)
FIRSTTRACK_PATH = "/home/robotics/catkin_ws/src/imp_tracker/data/seq2/firsttrack.txt"
GROUNDTRUTH_PATH = "/home/robotics/catkin_ws/src/imp_tracker/data/seq2/groundtruth.txt"
IMAGE_FOLDER    = "/home/robotics/catkin_ws/src/imp_tracker/data/seq2/img"

# MobileNet-SSD model files (please ensure the file paths are correct)
PROTO_TXT = "/home/robotics/catkin_ws/src/imp_tracker/mobilenet_ssd/deploy.prototxt"
MODEL     = "/home/robotics/catkin_ws/src/imp_tracker/mobilenet_ssd/mobilenet_iter_73000.caffemodel"
CONF_THRESHOLD = 0.5

# NUSNET ID (please change to your actual ID)
NUSNET_ID = "E1499298"

def load_bbox_from_file(filepath):
    """
    Read the initial bounding box from firsttrack.txt, format: x,y,width,height
    """
    with open(filepath, "r") as f:
        line = f.readline().strip()
    parts = line.split(',')
    bbox = list(map(float, parts))
    return bbox

def load_groundtruth(filepath):
    """
    Read the ground truth bounding boxes for each frame from groundtruth.txt, format: x,y,width,height
    """
    gt_bboxes = []
    with open(filepath, "r") as f:
        for line in f:
            parts = line.strip().split(',')
            if len(parts) >= 4:
                bbox = list(map(float, parts))
                gt_bboxes.append(bbox)
    return gt_bboxes

def detect_object(frame, net):
    """
    Use MobileNet-SSD to detect objects in the image, return the first bounding box that meets the confidence threshold (format: [x, y, width, height])
    """
    (h, w) = frame.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)),
                                 scalefactor=0.007843,
                                 size=(300, 300),
                                 mean=127.5)
    net.setInput(blob)
    detections = net.forward()
    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > CONF_THRESHOLD:
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            x1, y1, x2, y2 = box
            return [float(x1), float(y1), float(x2 - x1), float(y2 - y1)]
    return None

def create_detection_msg(bbox, stamp):
    """
    Construct a Detection2D message based on the given bbox [x, y, width, height].
    In BoundingBox2D, the center is (x+width/2, y+height/2) and size is (width, height)
    """
    det_msg = Detection2D()
    det_msg.header.stamp = stamp
    det_msg.header.frame_id = "camera"  # Set the frame_id according to your situation

    bb = BoundingBox2D()
    x, y, w, h = bbox
    bb.center.x = x + w / 2.0
    bb.center.y = y + h / 2.0
    bb.size_x = w
    bb.size_y = h
    det_msg.bbox = bb
    return det_msg

def reinitialize_tracker_and_kalman(first_bbox, image_folder, image_files):
    """
    Reinitialize the tracker and Kalman filter, load the first frame, and reinitialize using first_bbox
    """
    first_frame = cv2.imread(os.path.join(image_folder, image_files[0]))
    if first_frame is None:
        rospy.logerr("Unable to read the first frame during reinitialization")
        return None, None, None

    # Reinitialize CSRT Tracker
    if hasattr(cv2, 'TrackerCSRT_create'):
        tracker = cv2.TrackerCSRT_create()
    else:
        tracker = cv2.legacy.TrackerCSRT_create()
    init_bbox = tuple(map(int, first_bbox))
    tracker.init(first_frame, init_bbox)

    # Reinitialize Kalman filter
    kalman = cv2.KalmanFilter(8, 4)
    kalman.transitionMatrix = np.array([
        [1, 0, 0, 0, 1, 0, 0, 0],
        [0, 1, 0, 0, 0, 1, 0, 0],
        [0, 0, 1, 0, 0, 0, 1, 0],
        [0, 0, 0, 1, 0, 0, 0, 1],
        [0, 0, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 1]
    ], np.float32)
    kalman.measurementMatrix = np.array([
        [1, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 0, 0]
    ], np.float32)
    kalman.processNoiseCov = np.eye(8, dtype=np.float32) * 1e-2
    kalman.measurementNoiseCov = np.eye(4, dtype=np.float32) * 1e-1
    kalman.errorCovPost = np.eye(8, dtype=np.float32)
    kalman.statePost = np.array([first_bbox[0], first_bbox[1], first_bbox[2], first_bbox[3],
                                 0, 0, 0, 0], np.float32)

    rospy.loginfo("Reinitialize tracker and Kalman filter")
    return tracker, kalman, first_frame

def main():
    rospy.init_node('imp_tracker_node')

    # Initialize publishers
    viz_pub   = rospy.Publisher("/me5413/viz_output", Image, queue_size=1)
    gt_pub    = rospy.Publisher("/me5413/groundtruth", Detection2D, queue_size=1)
    track_pub = rospy.Publisher("/me5413/track", Detection2D, queue_size=1)
    id_pub    = rospy.Publisher("/me5413/nusnetID", String, queue_size=1)

    bridge = CvBridge()

    # Load initial bounding box and ground truth bounding boxes sequence
    first_bbox = load_bbox_from_file(FIRSTTRACK_PATH)
    gt_bboxes = load_groundtruth(GROUNDTRUTH_PATH)

    # Get list of images (sorted by name)
    image_files = sorted([f for f in os.listdir(IMAGE_FOLDER) if f.lower().endswith(('.jpg', '.png'))])
    if len(image_files) == 0:
        rospy.logerr("No images found in the specified directory")
        return

    # Load the first frame, initialize tracker and Kalman filter
    tracker, kalman, _ = reinitialize_tracker_and_kalman(first_bbox, IMAGE_FOLDER, image_files)
    if tracker is None or kalman is None:
        return

    # Load MobileNet-SSD model for object detection when tracking fails
    net = cv2.dnn.readNetFromCaffe(PROTO_TXT, MODEL)

    rate = rospy.Rate(15)  # Adjust loop frequency as needed
    img_idx = 0
    total_imgs = len(image_files)

    rospy.loginfo("imp_tracker_node started, beginning loop to publish data")
    while not rospy.is_shutdown():
        img_file = image_files[img_idx]
        img_path = os.path.join(IMAGE_FOLDER, img_file)
        frame = cv2.imread(img_path)
        if frame is None:
            rospy.logwarn("Unable to read image {}, skipping".format(img_path))
            img_idx = (img_idx + 1) % total_imgs
            rate.sleep()
            continue

        current_time = rospy.Time.now()

        # Kalman filter prediction
        predicted = kalman.predict()

        # Update object location using CSRT Tracker
        ok, bbox = tracker.update(frame)
        if ok:
            # Tracking succeeded, use tracker result to correct Kalman filter
            measurement = np.array(bbox, np.float32)
            kalman.correct(measurement)
            fused_bbox = kalman.statePost[:4].tolist()
            rospy.loginfo("CSRT tracking succeeded, bbox: {} fused: {}".format(bbox, fused_bbox))
        else:
            rospy.logwarn("CSRT tracking failed, attempting detection using MobileNet-SSD")
            detection_bbox = detect_object(frame, net)
            if detection_bbox is not None:
                measurement = np.array(detection_bbox, np.float32)
                kalman.correct(measurement)
                fused_bbox = kalman.statePost[:4].tolist()
                rospy.loginfo("SSD detected object, bbox: {} fused: {}".format(detection_bbox, fused_bbox))
                # Reinitialize CSRT Tracker
                if hasattr(cv2, 'TrackerCSRT_create'):
                    tracker = cv2.TrackerCSRT_create()
                else:
                    tracker = cv2.legacy.TrackerCSRT_create()
                tracker.init(frame, tuple(map(int, detection_bbox)))
            else:
                fused_bbox = predicted[:4].tolist()
                rospy.logwarn("SSD did not detect object either, using Kalman predicted bbox: {}".format(fused_bbox))

        # Draw the tracked bounding box on the image (green)
        x, y, w, h = map(int, fused_bbox)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, "Tracked", (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # If there is a ground truth bounding box for the current frame, draw it (red) and publish Detection2D message
        if img_idx < len(gt_bboxes):
            gt_bbox = gt_bboxes[img_idx]
            x_gt, y_gt, w_gt, h_gt = map(int, gt_bbox)
            cv2.rectangle(frame, (x_gt, y_gt), (x_gt + w_gt, y_gt + h_gt), (0, 0, 255), 2)
            cv2.putText(frame, "GroundTruth", (x_gt, y_gt - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            gt_det_msg = create_detection_msg(gt_bbox, current_time)
            gt_pub.publish(gt_det_msg)

        # Publish the Detection2D message for the tracked object
        track_det_msg = create_detection_msg(fused_bbox, current_time)
        track_pub.publish(track_det_msg)

        # Convert the current image with drawings to ROS Image message and publish
        img_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp = current_time
        viz_pub.publish(img_msg)

        # Publish NUSNET ID information
        id_msg = String(data=NUSNET_ID)
        id_pub.publish(id_msg)

        # Update image index, loop through images
        img_idx = (img_idx + 1) % total_imgs

        # When starting a new cycle (i.e., img_idx == 0), reinitialize tracker and Kalman filter
        if img_idx == 0:
            tracker, kalman, _ = reinitialize_tracker_and_kalman(first_bbox, IMAGE_FOLDER, image_files)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

