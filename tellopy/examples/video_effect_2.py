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
        self.drone = tellopy.Tello(port=port, tello_ip=tello_ip, interface=interface, video_port=video_port, name=name)
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


def main():
    global log_data
    drone_blue = tellopy.Tello(port=20001, tello_ip='192.168.10.1', interface='wlp0s20f3')
    drone_black = tellopy.Tello(port=20002, tello_ip='192.168.10.1', interface='wlx90de803fd246')

    try:
        print("===================1================")
        drone_blue.connect()
        drone_black.connect()
        drone_blue.wait_for_connection(10.0)
        drone_black.wait_for_connection(10.0)
        print("===================2================")

        retry = 3
        container_blue = None
        while container_blue is None and 0 < retry:
            retry -= 1
            try:
                container_blue = av.open(drone_blue.get_video_stream())
            except av.AVError as ave:
                print(ave)
                print('retry...')

        retry = 3
        container_black = None
        while container_black is None and 0 < retry:
            retry -= 1
            try:
                container_black = av.open(drone_black.get_video_stream())
            except av.AVError as ave:
                print(ave)
                print('retry...')

        # skip first 300 frames
        frame_skip = 300
        print("===================3================")
        while True:
            print("===================4================")
            for frame_blue, frame_black in zip(container_blue.decode(video=0), container_black.decode(video=0)):
                if 0 < frame_skip:
                    frame_skip = frame_skip - 1
                    continue
                start_time = time.time()
                image_blue = cv2.cvtColor(numpy.array(frame_blue.to_image()), cv2.COLOR_RGB2BGR)
                cv2.imshow('Blue', image_blue)
                cv2.imshow('Canny', cv2.Canny(image, 100, 200))
                cv2.waitKey(1)
                if frame_blue.time_base < 1.0 / 60:
                    time_base = 1.0 / 60
                else:
                    time_base = frame_blue.time_base
                frame_skip = int((time.time() - start_time) / time_base)

                if 0 < frame_skip:
                    frame_skip = frame_skip - 1
                    continue
                start_time = time.time()
                image_black = cv2.cvtColor(numpy.array(frame_black.to_image()), cv2.COLOR_RGB2BGR)
                cv2.imshow('Black', image_black)
                # cv2.imshow('Canny', cv2.Canny(image, 100, 200))
                cv2.waitKey(1)
                if frame_black.time_base < 1.0 / 60:
                    time_base = 1.0 / 60
                else:
                    time_base = frame_black.time_base
                frame_skip = int((time.time() - start_time) / time_base)

    except Exception as ex:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        print(ex)
    finally:
        drone_blue.quit()
        drone_black.quit()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    # main()
    rospy.init_node('tello', anonymous=True)
    tello_blue = TelloController(port=20000, interface='wlp0s20f3', video_port=5038, video=0, name='blue')
    tello_black = TelloController(port=20001, interface='wlx90de803fd246', video_port=7038, video=0, name='black')

    thread_blue = threading.Thread(target=tello_blue)
    thread_black = threading.Thread(target=tello_black)
    thread_blue.start()
    thread_black.start()
