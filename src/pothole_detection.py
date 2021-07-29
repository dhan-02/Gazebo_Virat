#!/usr/bin/env python2
from math import sqrt
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import imutils
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped, Vector3Stamped, Pose
import tf2_geometry_msgs
from Gazebo_Virat.msg import Centres
from image_geometry import PinholeCameraModel as pcm
from tf.transformations import quaternion_from_euler
class Pothole_Detection:
    
    def __init__(self):
        self.bridge = CvBridge()
        self.virat_camera = pcm() 
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) 
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.Subscriber('/virat/camera1/image_raw', Image, self.callback_image)
        rospy.Subscriber('/virat/camera1/camera_info', CameraInfo, self.callback_info)
        self.pub = rospy.Publisher('/virat/camera_detected_points',Centres,queue_size=100)
        self.transform = TransformStamped()
    def callback_image(self,msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # cv2.waitKey(3)
        img = cv2.medianBlur(img,5)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mask_w = cv2.inRange(gray, 240, 255)
        cnts = cv2.findContours(mask_w, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        centres = Centres()
        try:
            self.transform = self.tf_buffer.lookup_transform("odom",
                                       "camera_link",
                                       rospy.Time(0))
            # print(self.transform)
            for c in cnts:                                
                if(cv2.contourArea(c) > 500):
                    x,y,w,h = cv2.boundingRect(c)
                    cv2.rectangle(img,(int(x+w/2),int(y+h/2)),(int(x+5+w/2),int(y+5+h/2)),(0,0,255),-1)
                    centres.x.append(int(x+w/2))
                    centres.y.append(int(y+h/2))
                    # org = (int(x),int(y))
                    # cv2.putText(img,str(int(cv2.contourArea(c))),org,cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255))
                    ray_pixel = self.virat_camera.projectPixelTo3dRay((int(x+w/2),int(y+h/2)))
                    q = quaternion_from_euler(np.math.pi/2,0,np.math.pi/2)
                    tr = TransformStamped()
                    tr.transform.rotation.x = q[0]
                    tr.transform.rotation.y = q[1]
                    tr.transform.rotation.z = q[2]
                    tr.transform.rotation.w = q[3]

                    ray_stamped = Vector3Stamped()
                    ray_stamped.vector.x = ray_pixel[0]
                    ray_stamped.vector.y = ray_pixel[1]
                    ray_stamped.vector.z = ray_pixel[2]

                    ray = tf2_geometry_msgs.do_transform_vector3(ray_stamped, tr)

                    t = TransformStamped()
                    t.transform.rotation.x = self.transform.transform.rotation.x
                    t.transform.rotation.y = self.transform.transform.rotation.y
                    t.transform.rotation.z = self.transform.transform.rotation.z
                    t.transform.rotation.w = self.transform.transform.rotation.w

                    vt = tf2_geometry_msgs.do_transform_vector3(ray, t)
                    # print(vt)
                    d = vt.vector.x**2 + vt.vector.y**2 + vt.vector.z**2
                    # print(d)

                    cx = self.transform.transform.translation.x
                    cy = self.transform.transform.translation.y
                    cz = self.transform.transform.translation.z

                    px = -(vt.vector.x*cz)/vt.vector.z + cx
                    py = -(vt.vector.y*cz)/vt.vector.z + cy





                    ray_pixel1 = self.virat_camera.projectPixelTo3dRay((int(x+w/2),int(y)))

                    ray_stamped1 = Vector3Stamped()
                    ray_stamped1.vector.x = ray_pixel1[0]
                    ray_stamped1.vector.y = ray_pixel1[1]
                    ray_stamped1.vector.z = ray_pixel1[2]

                    ray1 = tf2_geometry_msgs.do_transform_vector3(ray_stamped1, tr)

                    d1 = tf2_geometry_msgs.do_transform_vector3(ray1, t)

                    px1 = -(d1.vector.x*cz)/d1.vector.z + cx
                    py1 = -(d1.vector.y*cz)/d1.vector.z + cy

                    ray_pixel2 = self.virat_camera.projectPixelTo3dRay((int(x+w/2),int(y+h)))
                    ray_stamped2 = Vector3Stamped()
                    ray_stamped2.vector.x = ray_pixel2[0]
                    ray_stamped2.vector.y = ray_pixel2[1]
                    ray_stamped2.vector.z = ray_pixel2[2]

                    ray2 = tf2_geometry_msgs.do_transform_vector3(ray_stamped2, tr)

                    d2 = tf2_geometry_msgs.do_transform_vector3(ray2, t)

                    px2 = -(d2.vector.x*cz)/d2.vector.z + cx
                    py2 = -(d2.vector.y*cz)/d2.vector.z + cy





                    ray_pixel3 = self.virat_camera.projectPixelTo3dRay((int(x),int(y+h/2)))

                    ray_stamped3 = Vector3Stamped()
                    ray_stamped3.vector.x = ray_pixel3[0]
                    ray_stamped3.vector.y = ray_pixel3[1]
                    ray_stamped3.vector.z = ray_pixel3[2]

                    ray3 = tf2_geometry_msgs.do_transform_vector3(ray_stamped3, tr)

                    d3 = tf2_geometry_msgs.do_transform_vector3(ray3, t)

                    px3 = -(d3.vector.x*cz)/d3.vector.z + cx
                    py3 = -(d3.vector.y*cz)/d3.vector.z + cy

                    ray_pixel4 = self.virat_camera.projectPixelTo3dRay((int(x+w),int(y+h/2)))
                    ray_stamped4 = Vector3Stamped()
                    ray_stamped4.vector.x = ray_pixel4[0]
                    ray_stamped4.vector.y = ray_pixel4[1]
                    ray_stamped4.vector.z = ray_pixel4[2]

                    ray4 = tf2_geometry_msgs.do_transform_vector3(ray_stamped4, tr)

                    d4 = tf2_geometry_msgs.do_transform_vector3(ray4, t)

                    px4 = -(d4.vector.x*cz)/d4.vector.z + cx
                    py4 = -(d4.vector.y*cz)/d4.vector.z + cy

                    diameter1 = (px1-px2)**2 + (py1-py2)**2
                    diameter1 = sqrt(diameter1)

                    diameter2 = (px3-px4)**2 + (py3-py4)**2
                    diameter2 = sqrt(diameter2)

                    diameter = (diameter2+diameter1)/2;                   
                    print("centre: ",px,py)
                    print("diameter",diameter)
            cv2.imshow("img",img)
            cv2.waitKey(1)
            self.pub.publish(centres)

            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        
        
    def callback_info(self,info_msg):
        self.virat_camera.fromCameraInfo(info_msg)
    
   
if __name__ == '__main__':

    rospy.init_node('object_detection')    
    detector = Pothole_Detection()
    rospy.spin()
