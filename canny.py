import rospy, cv2, cv_bridge, numpy, math, datetime
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class Follower:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        # cv2.namedWindow('window', 1)

        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

        self.twist = Twist()
        self.err = 0
        self.turn_cmd = Twist()

        self.signal = False
        self.begin = 0

        self.maxerr = 80
        self.last_turn=0

        # self.twist.linear.x = 0.4
        # self.twist.angular.z = -10 / 45
        # self.cmd_vel_pub.publish(self.twist)
        # print("test init")

    def image_callback(self, msg):
        # To get the image from the camera and convert it to binary image by using opencv
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define the threshold of red and green
        lower_red = numpy.array([170, 43,46])
        upper_red = numpy.array([180, 255, 255])

        sensitivity = 30
        lower_white = numpy.array([0,0,255-sensitivity])
        upper_white = numpy.array([255,sensitivity,255])
        # lower_white = numpy.array([0,0,0],dtype=numpy.uint8)
        # upper_white = numpy.array([0, 0, 255],dtype=numpy.uint8)

        # Segment red line and green line from hsv image and convert it to  binary image
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        #mask = mask_red + mask_green
        mask = mask_white
        mask = cv2.medianBlur(mask, 1)
        mask = cv2.erode(mask, (3,3))



        # To restrict our search to the 100-row portion of
        # the image corresponding to about the 0.2m distance in front of the Turtlebot
        h, w, d = image.shape
        search_top =  h / 3
        search_bot = search_top + 30
        # search_left = w/3
        # serch_right =32*w/3
        width = 200
        dis = 0
        pad = 100

        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        mask[:, 0: w / 2 - width+dis] = 0
        mask[:, w / 2 + width+dis: w] = 0

        # To find the line
        # Computing the contours of interesting area of hsv image
        im2,contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        areas = []

        print len(contours)

        for cont in contours:
            area = cv2.contourArea(cont)
            if area > 1300:
                areas.append(area)
            print area

        # To find the optimal index of contour
        cv2.rectangle(image,(w/2-width+dis,search_top),(w/2+width+dis,search_bot),(0,255,0),1)

        areas.sort()
        index = len(areas) - 1
        print len(areas)
        cv2.drawContours(image, contours, index, (0, 0, 255), 3)



        if index >= 0:
            # Computing the centroid of the contours of binary image
            M = cv2.moments(contours[index])
            self.signal = True
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(image, (cx, cy), 20, (255, 0, 0), -1)
            # P-controller
            self.err = cx - w/2
            if(self.err>self.maxerr):
                self.err = self.maxerr
            elif(self.err<self.maxerr*-1):
                self.err = self.maxerr*-1
            self.twist.linear.x = 0.2
            self.twist.angular.z = -float(self.err) / 45
            self.cmd_vel_pub.publish(self.twist)

            self.begin = datetime.datetime.now().second
        else:
            # Define actions when the point is not be found
            # 5 HZ
            r = rospy.Rate(5)
            # let's go forward at 0.1 m/s
            # by default angular.z is 0 so setting this isn't required
            move_cmd = Twist()
            move_cmd.linear.x = 0.1


            if(self.last_turn!=0 and self.signal is False):
                if self.last_turn < 0:
                    self.turn_cmd.angular.z = math.radians(-60)
                elif self.last_turn > 0:
                    self.turn_cmd.angular.z = math.radians(60)
                self.cmd_vel_pub.publish(self.turn_cmd)

            if self.signal is True:
                if self.err < 0:
                    self.turn_cmd.angular.z = math.radians(30)
                elif self.err > 0:
                    self.turn_cmd.angular.z = math.radians(-30)

                self.cmd_vel_pub.publish(self.turn_cmd)
                now = datetime.datetime.now().second

                # Go forward about 0.2m, then stop
                if math.fabs(now - self.begin) > 2:
                    for x in range(0, 10):
                        self.cmd_vel_pub.publish(move_cmd)
                        r.sleep()
                    self.signal = False
                    self.last_turn = self.err


        cv2.imshow('window', image)
        cv2.waitKey(3)

if __name__ == '__main__':
    rospy.init_node('follow')
    follow = Follower()
    rospy.spin()
