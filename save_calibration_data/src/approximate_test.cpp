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
using namespace sensor_msgs;
using namespace nav_msgs;
using namespace message_filters;
class writeFile
{
  public:
    writeFile(std::string rootFilePath_)
    {
       begin = ros::Time::now();
       rootFilePath = rootFilePath_;
       odom_path = rootFilePath + "rec/Odo.rec";
       std::ofstream odom_rec(odom_path);
       odom_rec.close();
    }
    void writeData(const ImageConstPtr& image, const OdometryConstPtr& odom)
    {
       std::ofstream odom_rec(odom_path, std::ofstream::app);
       if(!odom_rec)
          std::cerr << "open file failed!" << std::endl;
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
      cv::imwrite(rootFilePath+"image/"+std::to_string(imageCount)+".bmp", cv_ptr->image);
      tf::Quaternion q(odom->pose.pose.orientation.x,
      odom->pose.pose.orientation.y,
      odom->pose.pose.orientation.z,
      odom->pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll,pitch,yaw;
      m.getRPY(roll, pitch, yaw);
      double x = odom->pose.pose.position.x;
      double y = odom->pose.pose.position.y;
      ros::Duration image_duration = image->header.stamp - begin;
      ros::Duration odom_duration = odom->header.stamp - begin;
      std::ostringstream ostring;
      ostring << imageCount << " " << odom->header.stamp.toSec() << " " << image->header.stamp.toSec()
               << " " << x << " " << y << " " << yaw << "\n";
      std::cout << ostring.str();
      odom_rec << ostring.str();
      odom_rec.close();
      imageCount++;
    }
  private:
    std::string odom_path;
    ros::Time begin;
    static int imageCount;
    std::string rootFilePath;
};
int writeFile::imageCount = 0;
void callback(const ImageConstPtr& image, const OdometryConstPtr& odom, writeFile w)
{
  // Solve all of perception here...
   std::cout << "OK" << std::endl;
   w.writeData(image, odom);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");
  ros::NodeHandle nh;
  writeFile w("/home/xiaopeng/calibration_dataset/");
  message_filters::Subscriber<Image> image_sub(nh, "/camera/rgb/image_raw", 1);
  message_filters::Subscriber<Odometry> odom_sub(nh, "/odom", 1);
  ros::Time begin = ros::Time::now();
  typedef sync_policies::ApproximateTime<Image, Odometry> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, odom_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, w));
  
  ros::spin();

  return 0;
}