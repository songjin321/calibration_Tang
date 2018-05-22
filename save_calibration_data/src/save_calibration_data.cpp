#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_datatypes.h>
#include <functional>
#include <boost/bind.hpp>
#include <sstream>
#include <message_filters/sync_policies/approximate_time.h>
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to subscribe
    message_filters::Subscriber<sensor_msgs::Image> image_sub(n_, "/camera/rgb/image_raw",1);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(n_, "/odom",1);
    sync.reset(new Sync(MySyncPolicy(10), image_sub, odom_sub));
    sync->registerCallback(boost::bind(&SubscribeAndPublish::callback,this,_1,_2));
    cv::namedWindow("image");
    begin = ros::Time::now();
    std::cerr << "creat node" << std::endl;
    odom_rec.open("/home/xiaopeng/calibration_dataset/rec/Odo.rec");
    if(!odom_rec)
      std::cerr << "open file failed!" << std::endl;
  }
  ~SubscribeAndPublish()
  {
    cv::destroyWindow("image");
    odom_rec.close();
  }
  void callback(const sensor_msgs::ImageConstPtr& msg, const nav_msgs::OdometryConstPtr& odom)
  {
      std::cerr << "enter callback function" << std::endl;
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      // save image 
      cv::imwrite("/home/xiaopeng/calibration_dataset/image/"+std::to_string(imageCount)+".bmp", cv_ptr->image);
      // Update GUI Window
      cv::imshow("image", cv_ptr->image);
      cv::waitKey(3);
      // save odom 
      tf::Quaternion q(odom->pose.pose.orientation.x,
      odom->pose.pose.orientation.y,
      odom->pose.pose.orientation.z,
      odom->pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll,pitch,yaw;
      m.getRPY(roll, pitch, yaw);
      double x = odom->pose.pose.position.x;
      double y = odom->pose.pose.position.y;
      ros::Duration image_duration = msg->header.stamp - begin;
      ros::Duration odom_duration = odom->header.stamp - begin;
      std::ostringstream ostring;
      ostring << imageCount << " " << image_duration.toSec() << " " << odom_duration.toSec()
               << " " << x << " " << y << " " << yaw << "\n";
      std::cout << ostring.str();
      odom_rec << ostring.str();
      imageCount++;
  }

private:
    ros::NodeHandle n_; 
    int imageCount = 0;
    std::ofstream odom_rec;
    ros::Time begin;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;
};//End of class SubscribeAndPublish
int main(int argc, char** argv)
{
  ros::init(argc, argv, "save_calibration_data");
  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;
  ros::spin();
  return 0;
}

