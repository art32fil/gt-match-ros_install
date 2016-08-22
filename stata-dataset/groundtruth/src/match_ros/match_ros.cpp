/*
 * bag_player.cpp
 *
 *      Author: Hordur Johannsson
 *
 * This is a simple example that reads a bag file directly from a file.
 * The idea is that we can synchronously process bag files and pass to the
 * vision SLAM system.
 *
 */
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

//#include <image_transport/image_transport.h>
//#include <image_transport/subscriber_filter.h>
#include <tf/message_filter.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <boost/foreach.hpp>

#include "groundtruth/groundtruth_utils.hpp"

#include <lcmtypes/bot_core.h>
#include <ConciseArgs>

#define FALSE 0
#define VERBOSE FALSE

using namespace std;
using namespace isam;
using namespace Eigen;
using namespace scanmatch;


class App
{
public:
  App(string walls_file,isam::Pose2d initPose, int galignFrequency,
    int64_t begin_timestamp, int64_t end_timestamp,
    bool show_images):
   begin_timestamp(begin_timestamp), end_timestamp(end_timestamp),
   show_images(show_images){
    lcmref_ = lcm_create(NULL);
    
    if (show_images){
      ros::NodeHandle nh;
      image_pub_ = nh.advertise<sensor_msgs::Image>("/pr2_camera", 2);
    }
    
    gt = new GroundTruth(lcmref_,walls_file,initPose);
    gt->setgalignFrequency(galignFrequency);

    //int qwidth =640; // hardcoded 
    //int qheight =480; // hardcoded 
    //  image_data = new uint8_t [qwidth*qheight];
    //image_data = (uint8_t*) malloc(640 * 480*4);
  };
   
  lcm_t* get_lcmref(){return lcmref_;}
  //uint8_t* image_data ;
  GroundTruth* gt;
  ros::Publisher image_pub_;
  
  int64_t begin_timestamp;
  int64_t end_timestamp;
  bool show_images;
  
private:
  lcm_t* lcmref_ ;
  
};


/*
 Header header
  uint32 seq
  time stamp
  string frame_id
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data

640 w
480 h
bayer_grbg8 e
 big
640 st
307200 size

*/ 



void imageCb(const sensor_msgs::ImageConstPtr& msg,  App *app)
{
  if (app->show_images){
  /*
  ROS_INFO("Image callback");
  cout << msg->width << " w\n";
  cout << msg->height << " h\n";
  cout << msg->encoding << " e\n";
  cout << msg->is_bigendian << " big\n";
  cout << msg->step << " st\n";
  cout << msg->data.size() << " size\n";
  */
  app->image_pub_.publish(*msg); 
  
//  int pause;
//  cin >> pause;
//  bot_core_image_t img;
//  int64_t  msg_utime = (int64_t) floor(msg->header.stamp.toSec()  * 1E6);
//   img.utime = msg_utime;
//   img.width = msg->width;
//   img.height =msg->height;
//   img.row_stride = msg->width; // guess, check me  
//   img.size = 0;//msg->width*msg->height;  
//   img.nmetadata=0;
//   img.metadata=NULL;  
//   img.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY; 
//   copy(msg->data.begin(), msg->data.end(), gt->image_data);
//   //img.data = image_data;
//   img.data=NULL;
//   bot_core_image_t_publish(gt->get_lcmref(), "KINECT_RGB", &img);
//  delete image_data;
  }
}


void laserCb(const sensor_msgs::LaserScanConstPtr& scan,  App *app)
{
  int64_t  msg_utime = (int64_t) floor(scan->header.stamp.toSec()  * 1E6);

  if (msg_utime < app->begin_timestamp){
    cout << msg_utime << " is before the specified begin_timestamp ["<< app->begin_timestamp <<"]\n";
    cout << "RETURNING\n";
    return;
  }else if (msg_utime > app->end_timestamp){
    cout << msg_utime << " is after the specified end_timestamp - considering exiting here ["<< app->end_timestamp <<"]\n";
    cout << "RETURNING\n";
    return;
  }
  
  //ROS_INFO("Laser callback");
  
  cout << "================================\n";
  cout << msg_utime << " is time\n";  
  double timestamp = scan->header.stamp.toSec();
  int nranges = scan->ranges.size();
  
  /*
  cout << scan->angle_min << " w\n";
  cout << scan->angle_max << " h\n";
  cout << scan->angle_increment << " e\n";
  cout << scan->time_increment << " big\n";
  cout << scan->scan_time << " st\n";
  cout << scan->range_min << " min\n";
  cout << scan->range_max << " max\n";
  cout << nranges << " nranges\n";
  */
  
  double validBeamAngles[] ={scan->angle_min, scan->angle_max};// not sure about this...
  PointCloudPtr p1 = app->gt->align.convertToPointCloud(scan->ranges, 29.7, scan->angle_min, scan->angle_increment, validBeamAngles);
  app->gt->doGT(p1,msg_utime, scan->ranges,
      scan->angle_min, scan->angle_increment);    
}

void imuCb(const sensor_msgs::ImuConstPtr& imu)
{
  //ROS_INFO("IMU callback");
}

void odometryCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& odometry)
{
  //ROS_INFO("Odometry callback");
}

int main(int argc, char** argv)
{
  cout << "      2nd floor:\n"; // example path
  cout << "     walls_file: walls_2nd_floor\n";
  cout << "      log_fname: /media/Seagate/pr2/2012-04-06-11-15-29.bag\n";
  cout << "begin_timestamp: 1333736129321900\n";  
  cout << "init_x=29.30733458\n";
  cout << "init_y=129.694331\n";
  cout << "init_t=0.406198\n\n";  
  
  ConciseArgs parser(argc, argv, "match laser to map");
  string walls_file ="walls_2nd_floor";
  string log_fname ="/media/Seagate/pr2/2012-04-06-11-15-29.bag";
  int galignFrequency=150; // usually 150, use 800 to leaf through a log:
  int64_t begin_timestamp=0;
  int64_t end_timestamp =2335449257673990; // time in 2042  i.e. very far forward
  // old pr2 data set - goes with 2012-04-06-11-15-29.bag
  double init_x=29.30733458 ;
  double init_y=129.694331;
  double init_t=0.406198 ;   
  bool show_images= false; 
  parser.add(walls_file, "w", "walls_file", "text file of wall segments");
  parser.add(log_fname, "l", "log_fname", "ROS bag filename");
  parser.add(begin_timestamp, "b", "begin_timestamp", "Begin timestamp eg 1307727077792229");
  parser.add(end_timestamp, "e", "end_timestamp", "End timestamp eg 1307727077792229");
  parser.add(init_x, "x", "init_x", "Begin x pos");
  parser.add(init_y, "y", "init_y", "Begin y pos");
  parser.add(init_t, "t", "init_t", "Begin t pos");
  parser.add(galignFrequency, "g", "galign_freq", "Frequency to globally align a pose. Typical: 150 (~4sec), Sparse: 800 (20sec)");  
  parser.add(show_images, "s", "show_images", "Publish camera images to ROS to observe the location [requires a roscore]");    
  parser.parse();
  cout << galignFrequency << " is galignFrequency\n";
  cout << begin_timestamp << " is begin_timestamp\n";
  cout << end_timestamp << " is end_timestamp\n";
  cout << walls_file << " is walls_file\n";    
  cout << log_fname << " is log_fname\n";    
  cout << init_x << " is init_x\n";    
  cout << init_y << " is init_y\n";    
  cout << init_t << " is init_t\n";
  cout << show_images << " is show_images\n";      
  
  ros::init(argc, argv, "match_ros");
  
  ros::Time::init();
  rosbag::Bag bag;
  bag.open( log_fname , rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back("/base_scan");
  topics.push_back("/robot_pose_ekf/odom_combined");
  topics.push_back("/camera/rgb/image_raw");
  topics.push_back("/wide_stereo/left/image_raw");
  topics.push_back("/tf");
  topics.push_back("/torso_lift_imu/data");

  tf::Transformer transformer;
  
  isam::Pose2d initPose(init_x, init_y, init_t);   
  App app= App(walls_file,initPose,galignFrequency,begin_timestamp,end_timestamp,show_images);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    if (m.getTopic() == "/base_scan")
    {
      sensor_msgs::LaserScan::ConstPtr scan = m.instantiate<sensor_msgs::LaserScan>();
      if (scan != NULL)
        laserCb(scan, &app);
    }
    else if (m.getTopic() == "/robot_pose_ekf/odom_combined")
    {
      geometry_msgs::PoseWithCovarianceStamped::ConstPtr odometry = m.instantiate<geometry_msgs::PoseWithCovarianceStamped>();
      if (odometry != NULL)
        odometryCb(odometry);
    }
    else if (m.getTopic() == "/torso_lift_imu/data")
    {
      sensor_msgs::Imu::ConstPtr imu = m.instantiate<sensor_msgs::Imu>();
      if (imu != NULL)
        imuCb(imu);
    }
    else if (m.getTopic() == "/camera/rgb/image_raw")
    {
      sensor_msgs::Image::ConstPtr image = m.instantiate<sensor_msgs::Image>();
      if (image != NULL)
        imageCb(image, &app);
    }
    else if (m.getTopic() == "/wide_stereo/left/image_raw")
    {
      sensor_msgs::Image::ConstPtr image = m.instantiate<sensor_msgs::Image>();
      if (image != NULL)
        imageCb(image, &app);
    }
    else if (m.getTopic() == "/tf")
    {
      // These are the PR2 transformation we are interested in
      //  odom_combined -> base_link
      //  base_link -> imu_link
      //  base_link -> base_laser_link
      //  base_link -> openni_rgb_optical_frame
      //  base_link -> wide_stereo_optical_frame
      tf::tfMessage::ConstPtr tf_msg = m.instantiate<tf::tfMessage>();

      for (unsigned int i = 0; i < tf_msg->transforms.size(); ++i)
      {
        tf::StampedTransform transform;
        tf::transformStampedMsgToTF(tf_msg->transforms[i], transform);
        try 
        {
          transformer.setTransform(transform);
        } 
        catch (tf::TransformException& ex)
        {
          std::string temp = ex.what();
          ROS_ERROR("Failure to set recieved transform from %s to %s with error: %s\n", 
             tf_msg->transforms[i].child_frame_id.c_str(), 
             tf_msg->transforms[i].header.frame_id.c_str(), temp.c_str());
        }
      }
   
      // Lookup a transform
      try
      {
        tf::StampedTransform transform;
        transformer.lookupTransform("/base_link", "/base_laser_link", ros::Time(0), transform);
        tf::Vector3 t = transform.getOrigin();
        if (VERBOSE) std::cout << "/base_link -> /base_laser_link: " << t.x() << " " << t.y() << " " << t.z() << std::endl;        

        transformer.lookupTransform("/odom_combined", "/base_link", ros::Time(0), transform);
        t = transform.getOrigin();
        if (VERBOSE) std::cout << "/odom_combined -> /base_link: " << t.x() << " " << t.y() << " " << t.z() << std::endl;        
      } 
      catch (tf::TransformException ex) 
      {
         ROS_ERROR("lookupTransform failed: %s", ex.what());
      }
    }
  }
  return 0;
}

