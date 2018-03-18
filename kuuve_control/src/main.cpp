/* Author : Hwancheol, Donghee */
/* Date : 2017-05-19 3:00 */

/* Main Controller Node */

#include <iostream>
#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Time.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#define MODE_VISION_BASE 0 // 비젼 - 차선인식 모드
#define MODE_VISION_PARKING 1 // 비젼 - 주차

#define MODE_LIDAR_NARROW 2 // 라이더 - 협로
#define MODE_LIDAR_CURVE 3 // 라이더 - 곡선 (협로보다 속도 감소)
#define MODE_LIDAR_STATIC 4 // 라이더 - 정적 (협로 + 방향 바꾸기)
#define MODE_LIDAR_DYNAMIC 5 // 라이더 - 동적 (협로 + 전방 탐지 범위 증가 + 최소 속력 0 + 스티어링은 비젼)
#define MODE_LIDAR_PARKING 6 // 라이더 - 주차 (곡선 + 방향 바꾸기 + 일정시간 정차 + 후진)
#define MODE_LIDAR_UTURN 7 // 라이더 - U턴

#define MODE_GPS_REPAIRING 8// 	 GPS - 복구 모드(우선순위 최상위)
#define MODE_VISION_CROSSLINE 9

#define UTURN_SECOND 9.0
#define PARKING_STEERING 10
#define PARKING_SPEED 5.0
#define PARKING_STEERING_TIME 3

#define PARKING_LEFT 0
#define PARKING_RIGHT 1
using namespace std;
int laser_flag = 0;
int uturn_flag = 0;
int stop_flag_ = 0;
int stop_flag2 = 0;
ros::Publisher ackermann_pub;
ros::Publisher mode_pub;
ros::Publisher flag_pub;
ros::Publisher pd_pub;

int narrow_flag = 0;
int curve_flag = 0;
int mode_val = 0;
int prev_mode_val = 0;
bool mode_lock = false;
/* for parking */
double parking_distance;
int parking_direction;

int base_speed;
int base_steer;
int vision_calc_flag = 1;
bool flags[10];
vector<int> steer_buffer;
void publish_mode(int mode) {
	std_msgs::Int32 msg;
	msg.data = mode;

	int temp = 0;
	if(mode_val == MODE_LIDAR_NARROW) {
		if(narrow_flag == 0)
		  temp = 1;
	}
	else if(mode_val == MODE_LIDAR_CURVE) {
		if(curve_flag == 0)
		  temp = 1;
	}
	if(temp = 0)
	  mode_pub.publish(msg);
}
void control_sec(ros::Duration _control_time, float _steering_angle, float _speed) {
  std_msgs::Int32 flag_msg;
	flag_msg.data = 0;
	flag_pub.publish(flag_msg);

	ackermann_msgs::AckermannDriveStamped ackermann_data;
	ackermann_data.drive.steering_angle = _steering_angle;
	ackermann_data.drive.speed = _speed;


	ros::Time start_time = ros::Time::now();
	ros::Duration time_control(_control_time);
	ros::Rate r(5);
	while (ros::Time::now() - start_time < time_control) {
		ackermann_pub.publish(ackermann_data);
		ros::spinOnce();
		cout << "steer: " << _steering_angle << " speed: " << _speed << endl;
		r.sleep();
	}
	ackermann_data.drive.steering_angle = 0.0f;
	ackermann_data.drive.speed = 0.0f;
	ackermann_pub.publish(ackermann_data);
	ros::spinOnce();
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
	ROS_INFO("mode : %d", mode_val);
	ackermann_msgs::AckermannDriveStamped ackermann_data;
	const int KP = 10;
	int count = 0;
	if (mode_val == MODE_LIDAR_DYNAMIC) {
		for (int i = 494; i < 584 ; i++) {
			if (msg->ranges[i] < 1.5)
				count++;
		}
		if (count > 20)
			laser_flag = 1;
		if (count < 5 && laser_flag == 1) {
			laser_flag = 0;
		  mode_val = MODE_VISION_BASE;
		  mode_lock = false;
		}
	}
	else if (mode_val == MODE_LIDAR_UTURN) {
		count = 0;
		for (int i = 494; i < 584; i++) {
			if (msg->ranges[i] < 3.8)
				count++;
		}
		if (count > 10)
			uturn_flag = 1;
		if(uturn_flag == 1) {
			control_sec(ros::Duration(UTURN_SECOND), -28, 5);
			control_sec(ros::Duration(0.5), 0, 0);
			mode_lock = false;
		}
	}
	else if (mode_val == MODE_LIDAR_NARROW) {
		double min_dist = msg->ranges[360];
		for(int i = 360; i < 720; i++){
			if(msg->ranges[i] < min_dist) {
				min_dist = msg->ranges[i];
			}
		}
		if(min_dist < 3.5) {
			narrow_flag = 1;
			publish_mode(MODE_LIDAR_NARROW);
		}
	}
	else if (mode_val == MODE_LIDAR_CURVE) {
		double min_dist = msg->ranges[360];
		for(int i = 360; i < 720; i++){
			if(msg->ranges[i] < min_dist) {
				min_dist = msg->ranges[i];
			}
		}
		if(min_dist < 3.5) {
			curve_flag = 1;
			publish_mode(MODE_LIDAR_NARROW);
		}
	}
	else if (mode_val == MODE_LIDAR_STATIC) {
		double min_dist_right = msg->ranges[540];
		for(int i = 540; i < 660; i++) {
			if(msg->ranges[i] < min_dist_right)
				min_dist_right = msg->ranges[i];
		}
		if(min_dist_right < 1.5) {
			control_sec(ros::Duration(1.0), -14, 5);
		}
		double min_dist_left = msg->ranges[420];
		for(int i = 420; i < 540; i++) {
			if(msg->ranges[i] < min_dist_left)
				min_dist_left = msg->ranges[i];
		}
		if(min_dist_left < 1.5) {
			control_sec(ros::Duration(1.0), 14, 5);
		}
	}
	else if (mode_val == MODE_LIDAR_PARKING) {
		double min_dist = msg->ranges[240];
		for(int i = 240; i < 840; i++){
			if(msg->ranges[i] < min_dist) {
				min_dist = msg->ranges[i];
			}
		}
		if(min_dist > 7.2)
			parking_direction = PARKING_RIGHT;
		else
			parking_direction = PARKING_LEFT;
		std_msgs::Int32 pd_msg;
		pd_msg.data = parking_direction;
		pd_pub.publish(pd_msg);

	}

}

void sign_callback(const std_msgs::Int32 sign) {
	if (!mode_lock && !flags[sign.data] && sign.data != MODE_LIDAR_STATIC) {
		mode_val = sign.data;
		mode_lock = true;
	}


	switch (mode_val) {
	case MODE_VISION_PARKING:
		if (!flags[MODE_VISION_PARKING])
			flags[MODE_VISION_PARKING] = true;
		break;
	case MODE_VISION_CROSSLINE:
		if (!flags[MODE_VISION_CROSSLINE])
			flags[MODE_VISION_CROSSLINE] = true;
		break;
	case MODE_LIDAR_NARROW:
		if (!flags[MODE_LIDAR_NARROW])
			flags[MODE_LIDAR_NARROW] = true;
		break;
	case MODE_LIDAR_CURVE:
		if (!flags[MODE_LIDAR_CURVE])
			flags[MODE_LIDAR_CURVE] = true;
		break;
	case MODE_LIDAR_UTURN:
		if (!flags[MODE_LIDAR_UTURN])
			flags[MODE_LIDAR_UTURN] = true;
		break;
	case MODE_LIDAR_DYNAMIC:
		if (!flags[MODE_LIDAR_DYNAMIC])
			flags[MODE_LIDAR_DYNAMIC] = true;
		break;
	}
}

void pid_callback(const ackermann_msgs::AckermannDriveStamped data) {
	if (mode_val == MODE_LIDAR_NARROW || mode_val == MODE_LIDAR_CURVE || mode_val == MODE_LIDAR_PARKING) {
		ackermann_pub.publish(data);
		if(MODE_LIDAR_PARKING) {
			steer_buffer.push_back(data.drive.steering_angle);
		}
	}
}
void gps_callback(const ackermann_msgs::AckermannDriveStamped data) {
	if (mode_val == MODE_GPS_REPAIRING)
		ackermann_pub.publish(data);
}
void vision_callback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
	ackermann_msgs::AckermannDriveStamped pub_msg;
	pub_msg.drive.steering_angle = msg->drive.steering_angle;
	pub_msg.drive.speed = msg->drive.speed;
	base_speed = msg->drive.speed;
	base_steer = msg->drive.steering_angle;
	if(mode_val == MODE_VISION_BASE || (mode_val == MODE_LIDAR_NARROW && narrow_flag == 0)|| (mode_val == MODE_VISION_CROSSLINE && stop_flag_ == 0) || (mode_val == MODE_LIDAR_CURVE && curve_flag == 0) || (mode_val == MODE_LIDAR_DYNAMIC)) {
	  if((mode_val == MODE_LIDAR_DYNAMIC) && laser_flag == 1)
			pub_msg.drive.speed = 0;
		ackermann_pub.publish(pub_msg);
  }
}
void crossline_callback(const std_msgs::Int32 stop_flag) {
	if(stop_flag.data == 1) {
	stop_flag_ = stop_flag.data;
	if (mode_val == MODE_VISION_CROSSLINE && stop_flag_ == 1) {
		control_sec(ros::Duration(0.5), 0, 5.0);
		control_sec(ros::Duration(3.5), 0, 0.0);
		control_sec(ros::Duration(3.5), 0, 10.0);
		mode_lock = false;
		mode_val = MODE_VISION_BASE;
		publish_mode(mode_val);
	}
	if(!mode_lock)
	  stop_flag_ = 0;
	}
}
void parking_line_callback(const std_msgs::Int32 lidar_flag) {
	if (mode_val == MODE_VISION_PARKING && lidar_flag.data == 1) {
		mode_lock = false;
		mode_val = MODE_LIDAR_PARKING;
		//publish_mode(mode_val);

		mode_lock = true;
	}
}
void parking_direction_callback(const std_msgs::Int32 pd_flag) {
	if (mode_val == MODE_VISION_PARKING && pd_flag.data == 1) {
		control_sec(ros::Duration(1.0), 0, 10);
		control_sec(ros::Duration(PARKING_STEERING_TIME), PARKING_STEERING, PARKING_SPEED);
	}
}
void finish_callback(const std_msgs::Int32 finish_flag) {
	if(mode_val == MODE_LIDAR_NARROW || mode_val == MODE_LIDAR_CURVE) {
		mode_val = 0;
 		mode_lock = false;
	}
	if(mode_val == MODE_LIDAR_PARKING) {
		ackermann_msgs::AckermannDriveStamped return_msg;
		control_sec(ros::Duration(10.0), 0, 0);
		control_sec(ros::Duration(1.0), 0, -5);
		while(!steer_buffer.empty()) {
			return_msg.drive.speed = -5;
			return_msg.drive.steering_angle = -(steer_buffer.back());
			steer_buffer.pop_back();
			ackermann_pub.publish(return_msg);
		}
		mode_val = 0;
		mode_lock = false;
	}
}
int main(int argc, char** argv) {
	ros::init(argc, argv, "Control_Manager");
	ros::NodeHandle nh;
	ros::Subscriber signSub = nh.subscribe("sign", 1, sign_callback);
	ros::Subscriber scanSub = nh.subscribe("scan", 10, laserCallback);
	ros::Subscriber baseSub = nh.subscribe("vision_ackermann", 10, vision_callback);
	ros::Subscriber pidSub = nh.subscribe("ackermann_lidar", 10, pid_callback);
	ros::Subscriber gpsSub = nh.subscribe("ackermann_gps", 10, gps_callback);
	ros::Subscriber finish_Sub = nh.subscribe("finish", 10, finish_callback);
	ros::Subscriber crosslineSub = nh.subscribe("stop_line", 10, crossline_callback);
	ros::Subscriber parkinglineSub = nh.subscribe("parking_detect", 10, parking_line_callback);
	ros::Subscriber parkingdirectionSub = nh.subscribe("parking_direction", 10, parking_direction_callback);
	ackermann_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann", 10);
	mode_pub = nh.advertise<std_msgs::Int32>("mode", 10);
	flag_pub = nh.advertise<std_msgs::Int32>("lane_flag", 10);
	pd_pub = nh.advertise<std_msgs::Int32>("parking_direction", 1);


	ros::spin();
	return 0;
}
