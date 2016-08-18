/*
 * explode_bag.cpp
 */

#include <iostream>
#include <fstream>
#include <map>
#include <vector>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

//#include <image_transport/image_transport.h>
//#include <image_transport/subscriber_filter.h>
#include <tf/message_filter.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/image_encodings.h>
//#include <pr2_mechanism_controllers/BaseOdometryState.h>
//#include <pr2_mechanism_controllers/Odometer.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <ConciseArgs>

#define VERBOSE false

using namespace std;

// Split off a new image folder every X seconds:
int image_folder_dsec = 100;
int image_counter=0;
int laser_counter=0;

// Convert from a ROS time stamp to microseconds.
inline uint64_t stamp_to_utime(ros::Time stamp) {
  return stamp.sec * 1E6 + stamp.nsec/1E3;
}
inline ros::Time utime_to_stamp(uint64_t utime) {
  return ros::Time(uint32_t(utime/1E6), uint32_t(utime%uint64_t(1E6) * 1000));
}

class App{
public:
  App(std::string log_fname): log_fname_(log_fname){
    std::string outfname =  log_fname + ".csv";
    outputfile.open ( outfname.c_str() ); //temp2.str().c_str()); // output is now xyz quat
  };

  std::ofstream outputfile;

  std::string log_fname_;
private:

};

///////////////////// ROS-to-csv files ///////////////////////////////////////////////////
void laserCb(const sensor_msgs::LaserScanConstPtr& msg,std::string channel,  App *app){
  ostringstream temp2;
  temp2 << channel <<","
      << msg->header.seq << ","
      << msg->header.stamp.toNSec() << ","
      << msg->header.frame_id << ","
      << msg->angle_min << ","
      << msg->angle_max << ","
      << msg->angle_increment << ","
      << msg->scan_time << ","
      << msg->range_min << ","
      << msg->range_max << ","
      << msg->ranges.size();
  for (size_t i=0;i < msg->ranges.size(); i++){
    temp2 << "," << msg->ranges[i];
  }
  temp2 << "," << msg->intensities.size() ;
  for (size_t i=0;i < msg->intensities.size(); i++){
    temp2 << "," << msg->intensities[i];
  }
  app->outputfile << temp2.str() << endl;

  if (laser_counter%2000 ==0){
    int64_t  msg_utime = (int64_t) floor(msg->header.stamp.toSec()  * 1E6);
    std::cout << msg_utime<< ": " << laser_counter << " laser msgs written into text file\n";
  }
  laser_counter++;
}

void camerainfoCb(const sensor_msgs::CameraInfoConstPtr& msg,std::string channel,  App *app){
  ostringstream temp2;
  temp2 << channel <<","
      << msg->header.seq << ","
      << msg->header.stamp.toNSec() << ","
      << msg->header.frame_id << ","
      << msg->height << ","
      << msg->width << ","
      << msg->distortion_model;

  temp2 << "," << msg->D.size() ;
  for (size_t i=0;i < msg->D.size(); i++){ temp2 << "," << msg->D[i]; }

  temp2 << "," << msg->K.size() ;
  for (size_t i=0;i < msg->K.size(); i++){ temp2 << "," << msg->K[i]; }

  temp2 << "," << msg->R.size() ;
  for (size_t i=0;i < msg->R.size(); i++){ temp2 << "," << msg->R[i]; }

  temp2 << "," << msg->P.size() ;
  for (size_t i=0;i < msg->P.size(); i++){ temp2 << "," << msg->P[i]; }

  temp2 << "," << msg->binning_x
      << "," << msg->binning_y;

  temp2 << "," << msg->roi.x_offset << "," << msg->roi.y_offset
      << "," << msg->roi.height << "," << msg->roi.width;
  if (msg->roi.do_rectify){ temp2 << ",true";
  }else{ temp2 << ",false";}
  app->outputfile << temp2.str() << endl;
}

/*void pr2baseodoCb(const pr2_mechanism_controllers::BaseOdometryStateConstPtr& msg,std::string channel,  App *app){
  // no header in message
  ostringstream temp2;
  temp2 << channel <<","
      << msg->velocity.linear.x << "," << msg->velocity.linear.y << "," << msg->velocity.linear.z << ","
      << msg->velocity.angular.x << "," << msg->velocity.angular.y << "," << msg->velocity.angular.z ;
  temp2 << "," << msg->wheel_link_names.size() ;
  for (size_t i=0;i < msg->wheel_link_names.size(); i++){ temp2 << "," << msg->wheel_link_names[i]; }
  temp2 << "," << msg->drive_constraint_errors.size() ;
  for (size_t i=0;i < msg->drive_constraint_errors.size(); i++){ temp2 << "," << msg->drive_constraint_errors[i]; }
  temp2 << "," << msg->longitudinal_slip_constraint_errors.size() ;
  for (size_t i=0;i < msg->longitudinal_slip_constraint_errors.size(); i++){ temp2 << "," << msg->longitudinal_slip_constraint_errors[i]; }
  app->outputfile << temp2.str() << endl;
}

void pr2odometerCb(const pr2_mechanism_controllers::OdometerConstPtr& msg,std::string channel,  App *app){
  // no header in message
  ostringstream temp2;
  temp2 << channel <<"," << msg->distance << "," << msg->angle;
  app->outputfile << temp2.str() << endl;
}*/

void posewithcovCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg,std::string channel,  App *app){
  ostringstream temp2;
  temp2 << channel <<","
      << msg->header.seq << ","
      << msg->header.stamp.toNSec() << ","
      << msg->header.frame_id << ","
      << msg->pose.pose.position.x << "," << msg->pose.pose.position.y << "," << msg->pose.pose.position.z << ","
      << msg->pose.pose.orientation.x << "," << msg->pose.pose.orientation.y << "," << msg->pose.pose.orientation.z << "," << msg->pose.pose.orientation.w;
  temp2 << "," << msg->pose.covariance.size() ;
  for (size_t i=0;i < msg->pose.covariance.size(); i++){ temp2 << "," << msg->pose.covariance[i]; }
  app->outputfile << temp2.str() << endl;
}

void navodomCb(const nav_msgs::OdometryConstPtr& msg,std::string channel,  App *app){
  ostringstream temp2;
  temp2 << channel <<","
      << msg->header.seq << ","
      << msg->header.stamp.toNSec() << ","
      << msg->header.frame_id << ","
      << msg->child_frame_id << ","
      << msg->pose.pose.position.x << "," << msg->pose.pose.position.y << "," << msg->pose.pose.position.z << ","
      << msg->pose.pose.orientation.x << "," << msg->pose.pose.orientation.y << "," << msg->pose.pose.orientation.z << "," << msg->pose.pose.orientation.w;
  temp2 << "," << msg->pose.covariance.size() ;
  for (size_t i=0;i < msg->pose.covariance.size(); i++){ temp2 << "," << msg->pose.covariance[i]; }

  temp2 << "," << msg->twist.twist.linear.x << "," << msg->twist.twist.linear.y << "," << msg->twist.twist.linear.z << ","
      << msg->twist.twist.angular.x << "," << msg->twist.twist.angular.y << "," << msg->twist.twist.angular.z ;

  temp2 << "," << msg->twist.covariance.size() ;
  for (size_t i=0;i < msg->twist.covariance.size(); i++){ temp2 << "," << msg->twist.covariance[i]; }
  app->outputfile << temp2.str() << endl;
}

void imuCb(const sensor_msgs::ImuConstPtr& msg,std::string channel,  App *app){
  ostringstream temp2;
  temp2 << channel <<","
      << msg->header.seq << ","
      << msg->header.stamp.toNSec() << ","
      << msg->header.frame_id << ","
      << msg->orientation.x << "," << msg->orientation.y << "," << msg->orientation.z << "," << msg->orientation.w;

  temp2 << "," << msg->orientation_covariance.size() ;
  for (size_t i=0;i < msg->orientation_covariance.size(); i++){ temp2 << "," << msg->orientation_covariance[i]; }

  temp2 << "," << msg->angular_velocity.x << "," << msg->angular_velocity.y << "," << msg->angular_velocity.z;

  temp2 << "," << msg->angular_velocity_covariance.size() ;
  for (size_t i=0;i < msg->angular_velocity_covariance.size(); i++){ temp2 << "," << msg->angular_velocity_covariance[i]; }

  temp2 << "," << msg->linear_acceleration.x << "," << msg->linear_acceleration.y << "," << msg->linear_acceleration.z;

  temp2 << "," << msg->linear_acceleration_covariance.size() ;
  for (size_t i=0;i < msg->linear_acceleration_covariance.size(); i++){ temp2 << "," << msg->linear_acceleration_covariance[i]; }

  app->outputfile << temp2.str() << endl;
}

void boolCb(const std_msgs::BoolConstPtr& msg,std::string channel,  App *app){
  // no header
  ostringstream temp2;
  if (msg->data){ temp2 << channel <<",true";
  }else{ temp2 << channel <<",false";}
  app->outputfile << temp2.str() << endl;
}

void tfCb(const tf::tfMessageConstPtr& msg,std::string channel,  App *app){
  ostringstream temp2;
  temp2 << channel <<","
      << msg->transforms.size() ;
  for (size_t i=0;i < msg->transforms.size(); i++){
    temp2 << "," << msg->transforms[i].header.seq << ","
        << msg->transforms[i].header.stamp.toNSec() << ","
        << msg->transforms[i].header.frame_id << ","
        << msg->transforms[i].child_frame_id << ",";
    temp2 << msg->transforms[i].transform.translation.x << ","
        << msg->transforms[i].transform.translation.y << ","
        << msg->transforms[i].transform.translation.z << ","
        << msg->transforms[i].transform.rotation.x << ","
        << msg->transforms[i].transform.rotation.y << ","
        << msg->transforms[i].transform.rotation.z << ","
        << msg->transforms[i].transform.rotation.w;
  }
  app->outputfile << temp2.str() << endl;
}

void imageCb_text(const sensor_msgs::ImageConstPtr& msg,std::string channel,  App *app){
  // every field except the data
  ostringstream temp2;
  temp2 << channel <<","
      << msg->header.seq << ","
      << msg->header.stamp.toNSec() << ","
      << msg->header.frame_id << ","
      << msg->height << ","
      << msg->width << ","
      << msg->encoding << ","
      << ((int) msg->is_bigendian) << "," // uint8
      << msg->step;
  app->outputfile << temp2.str() << endl;
}

///////////////////// ROS images to png files ///////////////////////////////////////////////////
void imageCb(const sensor_msgs::ImageConstPtr& image,std::string channel,  App *app){
  //int64_t  msg_utime = (int64_t) floor(image->header.stamp.toSec()  * 1E6);
  int64_t  msg_utime = image->header.stamp.toNSec();

  if (VERBOSE){
    cout << channel << " channel\n";
    cout << image->width << " w | h " << image->height << "\n";
    cout << msg_utime << "\n";
  }

  namespace enc = sensor_msgs::image_encodings;
  cv_bridge::CvImageConstPtr image_ptr;
  try {
    // TODO support non-bayer images too
    image_ptr =  cv_bridge::toCvShare(image, enc::BAYER_BGGR8);// raw bayer
    //BGR8); color
    //MONO8); gray
  }catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  //  cv::imwrite("left.png", image_ptr->image);

  cv::Mat img_gray;
  cvtColor( image_ptr->image, img_gray, CV_BayerRG2BGR);
  std::stringstream smartpath;
  int64_t imfolder = floor( (double) msg_utime/1000000000/ image_folder_dsec)*image_folder_dsec;
  smartpath << "./" << app->log_fname_ << "images" << channel << "/" << imfolder ;


  if (!boost::filesystem::exists( smartpath.str() )){
    cout << smartpath.str() << " directory created\n";
    boost::filesystem::create_directories(smartpath.str());
  }
  std::stringstream  fname;
  fname <<  smartpath.str() << "/" << msg_utime << ".png";

  //cout << smartpath.str() << " is path\n";
  //cout << fname.str() << " is image\n";

  cv::imwrite(fname.str(), img_gray);
  if (image_counter%100 ==0){
    std::cout <<msg_utime <<": "<< image_counter << " images written" << "\n";
  }
  image_counter++;
}


int main(int argc, char** argv){
  ConciseArgs parser(argc, argv, "Explode Log File images");

  // 0 == other data
  // 1 == images
  // 2 == both
  int output_format = 0;
  std::string log_fname = "/media/passport1/data/pr2/2012/icra/2012-01-18-09-09-07.bag";
  parser.add(log_fname, "l", "log_fname", "ROS bag filename");
  parser.add(output_format, "o", "output_format", "0 other data | 1 images | 2 both");
  parser.parse();
  std::cout << log_fname << " is log_fname\n";
  std::cout << output_format << " is output_format\n";

  ros::init(argc, argv, "explode_bag");
  ros::Time::init();
  rosbag::Bag bag;
  bag.open(log_fname , rosbag::bagmode::Read);

  /*
  std::vector<std::string> topics;
  topics.push_back("/tf");
  topics.push_back("/base_scan");
  topics.push_back("/tilt_scan");
  topics.push_back("/wide_stereo/right/image_raw");
  topics.push_back("/wide_stereo/left/image_raw");
  topics.push_back("/wide_stereo/right/camera_info");
  topics.push_back("/wide_stereo/left/camera_info");
  // 2011 era files:
  topics.push_back("/wide_stereo_throttled/right/image_raw");
  topics.push_back("/wide_stereo_throttled/left/image_raw");
  topics.push_back("/wide_stereo_throttled/right/camera_info");
  topics.push_back("/wide_stereo_throttled/left/camera_info");
   */

  App* app;
  app= new App(log_fname);

  tf::Transformer transformer;
  //rosbag::View view(bag, rosbag::TopicQuery(topics));
  rosbag::View view(bag); // all msgs


  BOOST_FOREACH(rosbag::MessageInstance const m, view){
    //cout << m.getTopic() << " msg playing back\n";
    //cout << m.getDataType() << " types\n";

    if ((output_format==0)||(output_format==2)){
      if (m.getDataType() == "sensor_msgs/Image"){
        sensor_msgs::ImagePtr msg = m.instantiate<sensor_msgs::Image>();
        imageCb_text( msg , m.getTopic(), app);
      }else if (m.getDataType() == "sensor_msgs/CameraInfo"){
        sensor_msgs::CameraInfoPtr msg = m.instantiate<sensor_msgs::CameraInfo>();
        if (msg != NULL){ camerainfoCb(msg,m.getTopic(), app);}
      } else if (m.getDataType() == "tf/tfMessage") {
        tf::tfMessagePtr msg = m.instantiate<tf::tfMessage>();
        if (msg != NULL){ tfCb(msg,m.getTopic(), app);}
      } else if (m.getDataType() == "sensor_msgs/LaserScan"){
        sensor_msgs::LaserScan::ConstPtr msg = m.instantiate<sensor_msgs::LaserScan>();
        if (msg != NULL){ laserCb(msg,m.getTopic(), app);}
      /*} else if (m.getDataType() == "pr2_mechanism_controllers/BaseOdometryState"){
        pr2_mechanism_controllers::BaseOdometryState::ConstPtr msg = m.instantiate<pr2_mechanism_controllers::BaseOdometryState>();
        if (msg != NULL){ pr2baseodoCb(msg,m.getTopic(), app);}
      } else if (m.getDataType() == "pr2_mechanism_controllers/Odometer"){
        pr2_mechanism_controllers::Odometer::ConstPtr msg = m.instantiate<pr2_mechanism_controllers::Odometer>();
        if (msg != NULL){ pr2odometerCb(msg,m.getTopic(), app);}*/
      } else if (m.getDataType() == "geometry_msgs/PoseWithCovarianceStamped"){
        geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg = m.instantiate<geometry_msgs::PoseWithCovarianceStamped>();
        if (msg != NULL){ posewithcovCb(msg,m.getTopic(), app);}
      } else if (m.getDataType() == "nav_msgs/Odometry"){
        nav_msgs::Odometry::ConstPtr msg = m.instantiate<nav_msgs::Odometry>();
        if (msg != NULL){ navodomCb(msg,m.getTopic(), app);}
      } else if (m.getDataType() == "sensor_msgs/Imu"){
        sensor_msgs::Imu::ConstPtr msg = m.instantiate<sensor_msgs::Imu>();
        if (msg != NULL){ imuCb(msg,m.getTopic(), app);}
      } else if (m.getDataType() == "std_msgs/Bool"){
        std_msgs::Bool::ConstPtr msg = m.instantiate<std_msgs::Bool>();
        if (msg != NULL){ boolCb(msg,m.getTopic(), app);}
      } else {
        cout << m.getTopic() << " msg playing back\n";
        cout << m.getDataType() << " types\n";
        cout << "this message hasn't been listed!\n";
        int pause;
        cin >> pause;
      }
    }

    if ((output_format==1)||(output_format==2)){
      if (m.getDataType() == "sensor_msgs/Image"){
        sensor_msgs::ImagePtr msg = m.instantiate<sensor_msgs::Image>();
        if (msg != NULL){  imageCb( msg , m.getTopic(), app); }
      }
    }

  }

  return 0;
}



/*
      if (m.getTopic() == "/camera/rgb/camera_info"){
        sensor_msgs::CameraInfoPtr msg = m.instantiate<sensor_msgs::CameraInfo>();
        if (msg != NULL){ camerainfoCb(msg,m.getTopic(), app);}
      } else if (m.getTopic() == "/wide_stereo/left/camera_info") {
        sensor_msgs::CameraInfoPtr msg = m.instantiate<sensor_msgs::CameraInfo>();
        camerainfoCb(msg,m.getTopic(), app);}
      } else if (m.getTopic() == "/wide_stereo/right/camera_info") {
        sensor_msgs::CameraInfoPtr msg = m.instantiate<sensor_msgs::CameraInfo>();
        if (msg != NULL){ camerainfoCb(msg,m.getTopic(), app);}
      } else if (m.getTopic() == "/wide_stereo_throttled/left/camera_info") {
        sensor_msgs::CameraInfoPtr msg = m.instantiate<sensor_msgs::CameraInfo>();
        if (msg != NULL){ camerainfoCb(msg,m.getTopic(), app);}
      } else if (m.getTopic() == "/wide_stereo_throttled/right/camera_info") {
        sensor_msgs::CameraInfoPtr msg = m.instantiate<sensor_msgs::CameraInfo>();
        if (msg != NULL){ camerainfoCb(msg,m.getTopic(), app);}

      if (m.getTopic() == "/wide_stereo/left/image_raw") {
        sensor_msgs::ImagePtr image = m.instantiate<sensor_msgs::Image>();
        imageCb( image , m.getTopic(), app);

      } else if (m.getTopic() == "/wide_stereo/right/image_raw") {
        //sensor_msgs::ImagePtr image = m.instantiate<sensor_msgs::Image>();
        //imageCb( image , m.getTopic(), app);

      } else if (m.getTopic() == "/wide_stereo_throttled/left/image_raw") {
        sensor_msgs::ImagePtr image = m.instantiate<sensor_msgs::Image>();
        imageCb( image , m.getTopic(), app);
      }

 */
