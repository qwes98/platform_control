#include <ros/ros.h>
#include <iostream>
#include <std_msgs/String.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

using namespace std;

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann", 100);

    //Topic you want to subscribe
    sub_ = n_.subscribe("serial_data", 100, &SubscribeAndPublish::callback, this);
    publish();
  }

  void callback(const std_msgs::String::ConstPtr& input)
  {
    //cout << "subscribed speed data: " << input->data << endl;

    ackermann_msgs::AckermannDriveStamped output;
    output.drive.steering_angle = 2000;
    output.drive.speed = 30;

    pub_.publish(output);
  }

  void publish()
{
    while(true) {    
    ackermann_msgs::AckermannDriveStamped output;
    output.drive.steering_angle = 2;
    output.drive.speed = 0;
    pub_.publish(output);
    }

}

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "platform_control");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
