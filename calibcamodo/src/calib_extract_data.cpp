#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <iomanip>
#include <algorithm>
#include <iterator>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Core>

using namespace std;
using namespace cv;

double Time0 = 1513760000;

class ImageData{
public:
    bool updated;
    ImageData()
    {
        updated = false;
    }

    ~ImageData(){};

    void updateData(const sensor_msgs::Image& msg){
        while(updated){}
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
        cv_ptr->image.copyTo(img);
        frame_time = msg.header.stamp.toSec();
        updated = true;
    }

    void extractImage(Mat& _img, double& t)
    {
        img.copyTo(_img);
        t = frame_time;
        updated = false;
    }

private:
    double frame_time;
    Mat img;
};


class odoData{
public:
    bool updated;
    odoData()
    {
        updated = false;
        vOdo.setOnes();
    }

    ~odoData(){};

    void updateData(const nav_msgs::Odometry& msg){
        while(updated){}

        double x = msg.pose.pose.position.x;
        double y = msg.pose.pose.position.y;
        // get (x, y, yaw) from msg
        tf::Transform odom_tf;

        odom_tf.setOrigin(tf::Vector3(x, y, 0));
        odom_tf.setRotation(tf::Quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w));

        double roll, pitch, yaw;
        odom_tf.getBasis().getEulerYPR(yaw, pitch, roll);

        vOdo(0) = x;
        vOdo(1) = y;
        vOdo(2) = yaw;

        frame_time = msg.header.stamp.toSec();
        updated = true;
    }

    void extractOdo(Eigen::Vector3d& _odo, double& t)
    {
        _odo = vOdo;
        t = frame_time;
        updated = false;
    }

private:
    double frame_time;
    Eigen::Vector3d vOdo;
};

///* **************************************
// *************** image only **************
//****************************************/
int main(int argc, char **argv) {


    std::cout << "calib_extract_data node" << endl;
    //! Init ros
    ros::init(argc, argv, "calib_extract_data_sub");
    ros::start();
    ros::NodeHandle nh;

    ros::Rate rate(30);

    ImageData image;
    ros::Subscriber img_sub = nh.subscribe("/camera/image_raw", 1, &ImageData::updateData, &image);


    string ImageSavePath = "/home/wchen/Dataset/Calibation_odo_cam/bag/instrinsic/image2/";

    while (nh.ok()) {
        if(image.updated){
            Mat img;
            double img_time = 0.0;
            image.extractImage(img, img_time);

            stringstream ssTime;
            ssTime << setprecision(8) << (img_time - Time0);
            string sTime = ssTime.str();

//            putText(img, sTime, Point(img.cols/2, img.rows/2), FONT_HERSHEY_SIMPLEX,1, Scalar(255,255,0));

            cout << "image timestamp: " << sTime << " image size: " << img.cols << "x" << img.rows << "x" << img.channels() << endl;

            string fileName = ImageSavePath + sTime + ".bmp";
            imwrite(fileName, img);

//            imshow("image", img);
//            waitKey(5);
        }
        ros::spinOnce();
        rate.sleep();
    }
    img_sub.shutdown();
    return 0;
}

/* **************************************
 *************** odo only **************
****************************************/
//int main(int argc, char **argv) {

//    std::cout << "calib_extract_data node" << endl;
//    //! Init ros
//    ros::init(argc, argv, "calib_extract_data_sub");
//    ros::start();
//    ros::NodeHandle nh;

//    ros::Rate rate(50);

//    odoData odo;
//    ros::Subscriber odo_sub = nh.subscribe("/odom_localization_result", 1, &odoData::updateData, &odo);

//    string odoSavePath = "/home/wchen/Dataset/Calibation_odo_cam/bag/2017-12-20-16-58-02/rec/OdoOnly.rec";

//    ofstream odoFileW(odoSavePath.c_str());

//    int num = 0;

//    while (nh.ok()) {
//        if(odo.updated){

//            Eigen::Vector3d odo_rec;
//            double odo_time = 0.0;
//            odo.extractOdo(odo_rec, odo_time);

//            stringstream sssingleLine;
//            sssingleLine << num
//                         << " " << setprecision(8) << (odo_time - Time0)
//                         << " " << odo_rec(0)
//                         << " " << odo_rec(1)
//                         << " " << odo_rec(2);
//            string ssingleLine = sssingleLine.str();

//            odoFileW << ssingleLine << endl;

//            cout << num
//                 << " " << setprecision(8) << (odo_time - Time0)
//                 << " " << odo_rec(0)
//                 << " " << odo_rec(1)
//                 << " " << odo_rec(2) << endl;

//            num++;
//        }
//        ros::spinOnce();
//        rate.sleep();
//    }

//    odoFileW.close();
//    odo_sub.shutdown();
//    return 0;
//}

/* **************************************
 ****** Time synchronization ************
****************************************/

vector<string> SplitString(const string _str, const string _separator) {
    string str = _str;
    vector<string> vecstr_return;
    int cut_at;
    while ((cut_at = str.find_first_of(_separator)) != str.npos) {
        if (cut_at > 0) {
            vecstr_return.push_back(str.substr(0, cut_at));
        }
        str = str.substr(cut_at + 1);
    }
    if (str.length() > 0) {
        vecstr_return.push_back(str);
    }
    return vecstr_return;
}

bool ParseOdoData(const string& _str, vector<double>& vOdo) {
    vector<string> vec_str = SplitString(_str, " ");

    // fail
    if (vec_str[0] == "#") return false;

    vOdo.resize(4, -1);
    // read data
    vOdo.at(0) = atof(vec_str[1].c_str());
    vOdo.at(1)= atof(vec_str[2].c_str());
    vOdo.at(2) = atof(vec_str[3].c_str());
    vOdo.at(3) = atof(vec_str[4].c_str());
    return true;
}

bool ParseImgStampData(const string& _str, double& imgStamp) {
    vector<string> vec_str = SplitString(_str, ".");

    // read data
    string sTimeStamp = vec_str[0] + "." + vec_str[1];
    imgStamp = atof(sTimeStamp.c_str());
    return true;
}


void loadOdoOnly(string& Path, vector<vector<double>>& vvOdo, vector<double>& vOdoTime){

    ifstream odoFileIn(Path.c_str());
    string str_tmp;

    while(getline(odoFileIn, str_tmp)) {
        // read time info
        vector<double> odo_tmp;

        if (ParseOdoData(str_tmp, odo_tmp)){
            vvOdo.push_back(odo_tmp);
            vOdoTime.push_back(odo_tmp.at(0));
        }
    }
}

void loadImgStampOnly(string& Path, vector<double>& vImgTime, vector<string>& vsImgTime){

    ifstream imgFileIn(Path.c_str());
    string str_tmp;

    while(getline(imgFileIn, str_tmp)) {
        // read time info
        double iImgTime;
        if (ParseImgStampData(str_tmp, iImgTime)){
            vImgTime.push_back(iImgTime);
            vsImgTime.push_back(str_tmp);
            cout << str_tmp << " "
                 << setprecision(8) << iImgTime << endl;
        }
    }
}

//int main(int argc, char **argv) {


//    std::cout << "calib_extract_data node" << endl;

//    string ImageSavePath = "/home/wchen/Dataset/Calibation_odo_cam/bag/2017-12-20-16-58-02/image/";

//    string ImageReadPath = "/home/wchen/Dataset/Calibation_odo_cam/bag/2017-12-20-16-58-02/image_stamp/";

//    string odoReadPath = "/home/wchen/Dataset/Calibation_odo_cam/bag/2017-12-20-16-58-02/rec/OdoOnly.rec";

//    string imgStampFilePath = "/home/wchen/Dataset/Calibation_odo_cam/bag/2017-12-20-16-58-02/image_stamp.txt";

//    string odoWritePath = "/home/wchen/Dataset/Calibation_odo_cam/bag/2017-12-20-16-58-02/rec/Odo.rec";

//    string timeDiffSavePath = "/home/wchen/Dataset/Calibation_odo_cam/bag/2017-12-20-16-58-02/rec/TimeDiff.rec";
//    ofstream timeDiffFile(timeDiffSavePath.c_str());
//    if(timeDiffFile.is_open()){
//        timeDiffFile << "####" << endl;
//    }

//    ofstream odoFileW(odoWritePath.c_str());
//    if(odoFileW.is_open())
//    {
//        odoFileW << "# odometry info " << endl;
//        odoFileW << "# format: lp timeOdo timeCam x y theta" << endl;
//    }

//    vector<double> vImgStamp;
//    vector<string> vsImgStamp;
//    loadImgStampOnly(imgStampFilePath, vImgStamp, vsImgStamp);
//    cout << "img stamp loading done" << " size: " << vImgStamp.size() << endl;

//    vector<vector<double>> vvOdo;
//    vector<double> vOdoTime;
//    loadOdoOnly(odoReadPath, vvOdo, vOdoTime);
//    cout << "odo data loading done" << " size: " << vvOdo.size() << endl;

//    int num_odo = 0;


//    for(int iImg = 0; iImg < vImgStamp.size(); iImg ++){
//        double img_time = vImgStamp.at(iImg);

//        //find the odo data
//        vector<double> vodoTimeDiffBuff;
//        vodoTimeDiffBuff.resize(1000, -1);
//        int num_odo_tmp = num_odo;

//        int iOdoEnd = 0;
//        if(num_odo_tmp + 1000 - 1 > vvOdo.size())
//            iOdoEnd = vvOdo.size() - num_odo_tmp;
//        else
//            iOdoEnd = 1000;
//        for(int iOdo = 0; iOdo < iOdoEnd; iOdo++){
//            vodoTimeDiffBuff.at(iOdo) = fabs(vOdoTime.at(num_odo_tmp)- img_time);
//            num_odo_tmp++;
//        }

//        vector<double>::iterator smallest = min_element(vodoTimeDiffBuff.begin(), vodoTimeDiffBuff.end());

//        int dis = std::distance(vodoTimeDiffBuff.begin(), smallest);

//        if(*smallest <= 0.05)
//        {
//            vector<double>vOdo = vvOdo.at(num_odo + dis);

//            stringstream sssingleLine;
//            sssingleLine << iImg
//                         << " " << setprecision(8) << vOdo.at(0)
//                         << " " << setprecision(8) << img_time
//                         << " " << vOdo.at(1)
//                         << " " << vOdo.at(2)
//                         << " " << vOdo.at(3);
//            string ssingleLine = sssingleLine.str();

//            odoFileW << ssingleLine << endl;

//            stringstream ssImgRead;
//            ssImgRead << ImageReadPath << vsImgStamp.at(iImg);
//            string sImgRead = ssImgRead.str();

//            Mat img = imread(sImgRead);

//            stringstream ssNum;
//            ssNum << iImg ;
//            string sNum = ssNum.str();

//            string fileName = ImageSavePath + sNum + ".bmp";
//            imwrite(fileName, img);

//            cout << iImg
//                 << " " << setprecision(8) << vOdo.at(0)
//                 << " " << setprecision(8) << img_time
//                 << " " << vOdo.at(1)
//                 << " " << vOdo.at(2)
//                 << " " << vOdo.at(3)
//                 << endl;

//            timeDiffFile << setprecision(8) << (vOdo.at(0) - img_time) << endl;

//            num_odo += dis;
//        }
//    }

//    timeDiffFile.close();
//    return 0;
//}


