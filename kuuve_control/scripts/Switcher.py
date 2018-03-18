#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
import roslaunch

## Author : Hwancheol
## Data : 2017-05-18 01:05


uuid = 0
lidar_launch = 0
camera_launch = 0
webcam_launch = 0
gps_launch = 0

init_camera_launch = 0
init_gps_launch = 0
init_lidar_launch = 0
init_webcam_launch = 0
camera_switch = False
lidar_switch = False
gps_switch = False
webcam_switch = False

def kuuve_init() :
    global uuid
    global lidar_launch
    global camera_launch
    global webcam_launch
    global gps_launch
    global camera_switch
    global gps_switch
    global webcam_switch
    global init_camera_launch
    global init_gps_launch
    global init_lidar_launch
    global init_webcam_launch
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    lidar_launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/kuuve/catkin_ws/src/kuuve/pid_controller_lidar/launch/pid_controller_lidar.launch"])
    camera_launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/kuuve/catkin_ws/src/impro2_ackermann/launch/impro2.launch"])
    webcam_launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/kuuve/catkin_ws/src/ros_cam_six/launch/webcam_classifier.launch"])
    gps_launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/kuuve/catkin_ws/src/wpt_follower/launch/wpt_follower.launch"])

    init_camera_launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/kuuve/catkin_ws/src/avt_vimba_camera/launch/mono_camera.launch"])
    init_gps_launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/kuuve/catkin_ws/src/wpt_follower/launch/gps_imu.launch"])
    init_lidar_launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/kuuve/catkin_ws/src/lms1xx/launch/LMS1xx.launch"])
    init_webcam_launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/kuuve/catkin_ws/src/usb_cam/launch/usb_cam-test.launch"])

    init_camera_launch.start()
    init_gps_launch.start()
    init_lidar_launch.start()
    init_webcam_launch.start()
    camera_launch.start()
    webcam_launch.start()
    gps_launch.start()
    webcam_switch = True
    gps_switch = True

def switch(mode):
    global uuid
    global lidar_launch
    global camera_launch
    global camera_parking_launch
    global webcam_launch
    global gps_launch
    global camera_switch
    global lidar_switch
    global gps_switch
    global webcam_switch
    global camera_parking_switch

    ## Camera On
    if mode == 0 :
        if camera_switch == False :
            camera_launch.start()
            camera_switch = True
        if webcam_switch == False :
            webcam_launch.start()
            webcam_switch = True
    #Mode Camera Parking
    elif mode == 1 :
        if camera_switch == False :
            camera_launch.start()
            camera_switch = True
        if webcam_switch == True :
            webcam_launch.shutdown()
            webcam_switch = False

    elif mode >= 2 and mode <= 7 :
        if lidar_switch == False :
            lidar_launch.start()
            lidar_switch = True
        if webcam_switch == True :
            webcam_launch.shutdown()
            webcam_switch = False


    elif mode == 8 :
        if lidar_switch == True :
            lidar_launch.shutdown()
            lidar_switch = False
        if webcam_switch == True :
            webcam_launch.shutdown()
            webcam_switch = False
        if camera_switch == True :
            camera_launch.shutdown()
            camera_switch = False
    elif mode == 9 :
        if camera_switch == False :
            camera_launch.start()
            camera_switch = True
        if webcam_switch == True :
            webcam_launch.shutdown()
            webcam_switch == False
        if lidar_switch == True :
            lidar_launch.shutdown()
            lidar_switch = False
if __name__ == '__main__':
    rospy.init_node('Switcher', anonymous=True)
    kuuve_init();
    sub = rospy.Subscriber("mode", Int32, switch)
    rospy.spin()
    cameara_switch = True
