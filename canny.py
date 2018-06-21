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


    def image_callback(self, msg):
        # To get the image from the camera and convert it to binary image by using opencv
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        image = cv2.GaussianBlur(img,(11,11),2)
        image = cv2.Canny(image,8,200,3)

        sensitivity = 30
        lower_white = numpy.array([1])
        upper_white = numpy.array([255])

        mask_white = cv2.inRange(image, lower_white, upper_white)

        #mask = mask_red + mask_green
        mask = mask_white
        mask = cv2.medianBlur(mask, 1)
        mask = cv2.erode(mask, (3,3))


        # To restrict our search to the 100-row portion of
        # the image corresponding to about the 0.2m distance in front of the Turtlebot
        h, w = image.shape
        search_top =  h / 3
        search_bot = search_top + 15
        # search_left = w/3
        # serch_right =32*w/3
        width = 180
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

        x = 0
        y = 0

        cv2.rectangle(img,(w/2-width+dis,search_top),(w/2+width+dis,search_bot),(0,255,0),1)

        areas.sort()
        index = len(areas) - 1
        print len(areas)
        x = 0
        y = 0
        sum = 0
        cv2.drawContours(img, contours, index, (0, 0, 255), 3)
        for i in range(search_top,search_bot):
            for j in range(w/2-width+dis,w/2+width+dis):
                if(image[i][j]==255):
                    x += j
                    y += i
                    sum += 1


        if (sum>0):

            x /= sum
            y /= sum
            cv2.circle(img,(x,y),20,(255,0,0),-1)
            # # P-controller
            self.err = x - w/2
            self.twist.linear.x = 0.3
            self.twist.angular.z = -float(self.err) / 60
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

        cv2.imshow('window', image)
        cv2.imshow('window1',img)
        cv2.waitKey(3)

if __name__ == '__main__':
    rospy.init_node('follow')
    follow = Follower()
    rospy.spin()
