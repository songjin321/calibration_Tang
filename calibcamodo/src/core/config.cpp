#include "config.h"
#include "type.h"

#include <iostream>


using namespace std;
using namespace cv;

namespace calibcamodo {

//! IO
int Config::NUM_FRAME;
std::string Config::STR_FOLDERPATH_MAIN;
std::string Config::STR_FOlDERPATH_IMG;
std::string Config::STR_FILEPATH_ODO;
std::string Config::STR_FILEPATH_CAM;
std::string Config::STR_FILEPATH_CALIB;
std::string Config::STR_FILEPATH_ORBVOC;

//!Camera Intrinsics
int Config::IMAGE_WIDTH;
int Config::IMAGE_HEIGHT;
cv::Mat Config::CAMERA_MATRIX;
cv::Mat Config::DISTORTION_COEFFICIENTS;

//!Camera Extrinsic
cv::Mat Config::RVEC_BC = (Mat_<float>(3,1) << 0,0,0);
cv::Mat Config::TVEC_BC = (Mat_<float>(3,1) << 0,0,0);

//!Calibration Result for output
cv::Mat Config::rvec_b_c;
cv::Mat Config::tvec_b_c;
cv::Mat Config::T_b_c;

//! Dataset
double Config::DATASET_THRESH_KF_ODOLIN;
double Config::DATASET_THRESH_KF_ODOROT;
double Config::MARK_SIZE;

//! Solver
double Config::CALIB_ODOLIN_ERRR;
double Config::CALIB_ODOLIN_ERRMIN;
double Config::CALIB_ODOROT_ERRR;
double Config::CALIB_ODOROT_ERRRLIN;
double Config::CALIB_ODOROT_ERRMIN;

double Config::CALIB_AMKZ_ERRRZ;
double Config::CALIB_AMKZ_ERRMIN;
double Config::CALIB_AMKXY_ERRRZ;
double Config::CALIB_AMKXY_ERRMIN;

//! ROS PUBLISHER
double Config::MAPPUB_SCALE_RATIO;

void Config::InitConfig(std::string _strfolderpathmain) {

    STR_FOLDERPATH_MAIN = _strfolderpathmain;
    STR_FOlDERPATH_IMG  = _strfolderpathmain+"image/";
    STR_FILEPATH_ODO    = _strfolderpathmain+"rec/Odo.rec";
    STR_FILEPATH_CAM    = _strfolderpathmain+"config/CamConfig.yml";
    STR_FILEPATH_CALIB  = _strfolderpathmain+"config/CalibConfig.yml";

    FileStorage file(STR_FILEPATH_CALIB, cv::FileStorage::READ);
    file["NUM_FRAME"] >> NUM_FRAME;

    file["IMAGE_WIDTH"] >> IMAGE_WIDTH;
    file["IMAGE_HEIGHT"] >> IMAGE_HEIGHT;
    Mat cameramatrix;
    file["CAMERA_MATRIX"] >> cameramatrix;
    cameramatrix.convertTo(CAMERA_MATRIX, CV_32FC1);
    Mat distortion;
    file["DISTORTION_COEFFICIENTS"] >> distortion;
    distortion.convertTo(DISTORTION_COEFFICIENTS, CV_32FC1);

    Mat rvec_bc, tvec_bc;
    file["RVEC_BC"] >> rvec_bc;
    if(!rvec_bc.empty())
        rvec_bc.convertTo(RVEC_BC, CV_32FC1);
    file["TVEC_BC"] >> tvec_bc;
    if(!tvec_bc.empty())
        tvec_bc.convertTo(TVEC_BC, CV_32FC1);

//    cout << rvec_bc  << " " << tvec_bc << endl;

    file["MARK_SIZE"] >> MARK_SIZE;

    file["DATASET_THRESH_KF_ODOLIN"] >> DATASET_THRESH_KF_ODOLIN;
    file["DATASET_THRESH_KF_ODOROT"] >> DATASET_THRESH_KF_ODOROT;

    file["CALIB_ODOLIN_ERRR"] >> CALIB_ODOLIN_ERRR;
    file["CALIB_ODOLIN_ERRMIN"] >> CALIB_ODOLIN_ERRMIN;
    file["CALIB_ODOROT_ERRR"] >> CALIB_ODOROT_ERRR;
    file["CALIB_ODOROT_ERRRLIN"] >> CALIB_ODOROT_ERRRLIN;
    file["CALIB_ODOROT_ERRMIN"] >> CALIB_ODOROT_ERRMIN;
    file["CALIB_AMKZ_ERRRZ"] >> CALIB_AMKZ_ERRRZ;
    file["CALIB_AMKZ_ERRMIN"] >> CALIB_AMKZ_ERRMIN;
    file["CALIB_AMKXY_ERRRZ"] >> CALIB_AMKXY_ERRRZ;
    file["CALIB_AMKXY_ERRMIN"] >> CALIB_AMKXY_ERRMIN;

    file["MAPPUB_SCALE_RATIO"] >> MAPPUB_SCALE_RATIO;

    file["STR_FILEPATH_ORBVOC"] >> STR_FILEPATH_ORBVOC;
}

void Config::SetExtrinsics(Mat rvec_bc, Mat tvec_bc, Mat T_bc)
{
  rvec_bc.convertTo(rvec_b_c, CV_32FC1);
  tvec_bc.convertTo(tvec_b_c, CV_32FC1);
  T_bc.convertTo(T_b_c, CV_32FC1);
}

void Config::SavePara()
{
  FileStorage file(STR_FILEPATH_CAM, FileStorage::WRITE);
  file << "image_width" << IMAGE_WIDTH;
  file << "image_height" << IMAGE_HEIGHT;
  file << "camera_matrix" << CAMERA_MATRIX;
  file << "distortion_coefficients" << DISTORTION_COEFFICIENTS;
  file << "rvec_b_c" << rvec_b_c;
  file << "tvec_b_c" << tvec_b_c;
  file << "T_b_c" << T_b_c;
  file.release();
  cerr << "Saved" << std::endl;
}

}
