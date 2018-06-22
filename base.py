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

        self.hasFind=0
        self.findArrow=0
        self.timeFlag=0
        self.turnFlag=0
        self.startTime=0

    def Direction(self,up,down,left,right):
        disul=(up[0]-left[0])*(up[0]-left[0])+(up[1]-left[1])*(up[1]-left[1])
        disur=(up[0]-right[0])*(up[0]-right[0])+(up[1]-right[1])*(up[1]-right[1])
        disdl=(down[0]-left[0])*(down[0]-left[0])+(down[1]-left[1])*(down[1]-left[1])
        disdr=(down[0]-right[0])*(down[0]-right[0])+(down[1]-right[1])*(down[1]-right[1])
        if disul<=disur and disdl<=disdr:
            return "left"
        elif disul>=disur and disdl>=disdr:
            return "right"
        elif (disul>disur and disdl<disdr) or (disul<disur and disdl>disdr):
            if disul<disdr:
                return "right"
            else:
                return "left"

    def FindArrow(self,img):
            if  self.findArrow==0:
                gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
                tem_img = cv2.imread('tem_left.png')
                tem_gray = cv2.cvtColor(tem_img,cv2.COLOR_BGR2GRAY)
                ret, binary = cv2.threshold(gray,127,255,cv2.THRESH_BINARY)
                tem_ret, tem_binary = cv2.threshold(tem_gray,127,255,cv2.THRESH_BINARY)
                #binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 5, 7)
                #tem_binary = cv2.adaptiveThreshold(tem_gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 5, 7)
                binary, contours, hierarchy = cv2.findContours(binary,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                binary, tem_contours, hierarchy = cv2.findContours(tem_binary,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

                #print countours_list size (list element is numpy.ndarray)
                #print (len(tem_contours))
                pro=1
                min_pro=999
                min_num=-1

                for num in range(len(contours)):
                     if cv2.contourArea(contours[num])<3500 or cv2.contourArea(contours[num])>4000:
                        continue
                     pro=cv2.matchShapes(contours[num],tem_contours[0],1, 1.0)
                     print(pro,cv2.contourArea(contours[num]))
                     if pro<min_pro:
                        min_pro=pro;
                        min_num=num;
                #x, y, w, h =cv2.boundingRect(contours[0]);
                #cv2.rectangle(img,(x, y), (x+w, y+h) , (0,0,255),2);
                #print(contours[0]);
                #print(tem_contours[0]);
                if min_pro>0.38 and min_pro<0.45:
                    print("find arrow!!!",min_pro)
                    self.findArrow=1
                    pentagram = contours[min_num]
                    top = tuple(pentagram[:,0][pentagram[:,:,1].argmin()])
                    buttom = tuple(pentagram[:,0][pentagram[:,:,1].argmax()])
                    left = tuple(pentagram[:,0][pentagram[:,:,0].argmin()])
                    right = tuple(pentagram[:,0][pentagram[:,:,0].argmax()])
                    if self.Direction(top,buttom,left,right)=="right":
                        self.turnFlag=1
                    else:
                        self.turnFlag=-1
                    #cv2.drawContours(img,contours,-1,(0,0,255),3)
                    cv2.drawContours(img,contours[min_num],-1,(0,255,0),3)
                    print("turn :",self.turnFlag)
            else:
                if self.timeFlag==0:
                    self.timeFlag=1
                    self.startTime=datetime.datetime.now().second
                else:
                    passTime=datetime.datetime.now().second-self.startTime
                    if passTime>1:
                        self.hasFind=1
                #print("hasFinnd time:",passTime)

    def image_callback(self, msg):
        # To get the image from the camera and convert it to binary image by using opencv
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        image = cv2.GaussianBlur(img,(11,11),2)
        image = cv2.Canny(image, 80, 200,3)

        #hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        if self.hasFind ==0:
            self.FindArrow(img)




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

       # print len(contours)

        x = 0
        y = 0

        cv2.rectangle(img,(w/2-width+dis,search_top),(w/2+width+dis,search_bot),(0,255,0),1)

        areas.sort()
        index = len(areas) - 1
        #print len(areas)
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
            if(self.hasFind==0 and self.findArrow == 1):
                self.twist.linear.x = 0
                if(self.turnFlag==1):
                    self.twist.angular.z = -30/60
                elif(self.turnFlag==-1):
                    self.twist.angular.z = 30/60

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

