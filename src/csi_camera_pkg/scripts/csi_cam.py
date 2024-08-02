#!/usr/bin/env python
import rospy
import cv2
import datetime
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def open_csi_camera():
    pipeline = (
        'nvarguscamerasrc ! '
        'video/x-raw(memory:NVMM), width=640, height=480, framerate=30/1 ! '
        'nvvidconv ! '
        'video/x-raw, format=BGRx ! '
        'videoconvert ! '
        'video/x-raw, format=BGR ! '
        'appsink'
    )
    rospy.loginfo("GStreamer pipeline: %s", pipeline)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        rospy.logerr('Failed to open camera with pipeline: %s', pipeline)
    return cap

def generate_video_filename():
    return "video_{}.mp4".format(datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))

def record_video():
    rospy.init_node('camera_recorder', anonymous=True)
    cap = open_csi_camera()
    if not cap.isOpened():
        rospy.logerr('Camera did not open.')
        return

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fourcc = cv2.VideoWriter_fourcc(*'H264')
    out = None
    recording = False
    bridge = CvBridge()
    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logerr('Failed to capture frame')
            break

        image_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_pub.publish(image_msg)

        cv2.imshow('CSI Camera', frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord(' '):  # Space to start/stop recording
            if recording:
                out.release()
                rospy.loginfo("Video saved.")
                recording = False
            else:
                filename = generate_video_filename()
                out = cv2.VideoWriter(filename, fourcc, 30.0, (width, height))
                recording = True
                rospy.loginfo("Recording started...")

        if recording:
            out.write(frame)

        if key == ord('q'):  # 'q' to quit
            break

    if recording:
        out.release()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        record_video()
    except rospy.ROSInterruptException:
        pass
