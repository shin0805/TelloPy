import sys
import traceback
import tellopy
import av
import cv2
import os
import numpy
import time
import threading
import rospy
import rospkg
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np


class TelloController:

    def __init__(self, port=9000, tello_ip='192.168.10.1', interface=None, video_port=6038, video=0, name='tello'):
        self.name = name
        self.video = video
        self.drone = tellopy.Tello(port=port, tello_ip=tello_ip, interface=interface, video_port=video_port)
        self.drone.connect()
        self.drone.wait_for_connection(10.0)
        retry = 3
        self.container = None
        while self.container is None and 0 < retry:
            retry -= 1
            try:
                self.container = av.open(self.drone.get_video_stream())
            except av.AVError as ave:
                print(ave)
                print('retry...')
        # skip first 300 frames
        self.frame_skip = 300
        # self.drone.subscribe(self.drone.EVENT_FLIGHT_DATA, self.handler)
        # self.drone.subscribe(self.drone.EVENT_LOG_DATA, self.handler)

        # image
        self.rate = rospy.Rate(60)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('/tello_' + self.name + '/image_raw', Image, queue_size=10)

    def loop(self):
        for frame in self.container.decode(video=self.video):
            if 0 < self.frame_skip:
                self.frame_skip = self.frame_skip - 1
                continue
            start_time = time.time()
            # image = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)
            # cv2.imshow(self.name, image)
            # cv2.waitKey(1)
            # img publish
            img_msg = self.bridge.cv2_to_imgmsg(np.array(frame.to_image()), encoding="rgb8")
            img_msg.header.frame_id = "tello_camera_link"
            img_msg.header.stamp = rospy.Time.now()
            self.image_pub.publish(img_msg)
            if frame.time_base < 1.0 / 60:
                time_base = 1.0 / 60
            else:
                time_base = frame.time_base
            self.frame_skip = int((time.time() - start_time) / time_base)

        self.rate.sleep()

    def __call__(self):
        try:
            while not rospy.is_shutdown():
                self.loop()
        except Exception as e:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            traceback.print_exception(exc_type, exc_value, exc_traceback)
            print(e)
        finally:
            self.drone.quit()
            cv2.destroyAllWindows()

    def handler(self, event, sender, data, **args):
        drone = sender
        if self.name == 'blue':
            print('\033[94mblue ' + str(data) + '\033[0m')
        else:
            print(self.name + ' ' + str(data))

        if event is drone.EVENT_LOG_DATA:
            self.log_data = data
        elif event is drone.EVENT_FLIGHT_DATA:
            self.flight_data = data


if __name__ == '__main__':
    # main()
    rospy.init_node('tello', anonymous=True)
    tello_blue = TelloController(port=20000, interface='wlp0s20f3', video_port=5038, video=0, name='blue')
    tello_black = TelloController(port=20001, interface='wlx90de803fd246', video_port=7038, video=0, name='black')

    thread_blue = threading.Thread(target=tello_blue)
    thread_black = threading.Thread(target=tello_black)
    thread_blue.start()
    thread_black.start()
