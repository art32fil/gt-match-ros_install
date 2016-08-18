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
#include <boost/foreach.hpp>
#include "groundtruth/groundtruth_utils.hpp"
#include <ConciseArgs>

#define TRUE 1
#define VERBOSE TRUE

// Convert from a ROS time stamp to microseconds.
inline int64_t stamp_to_utime(ros::Time stamp) {
  return stamp.sec * 1E6 + stamp.nsec/1E3;
}

inline ros::Time utime_to_stamp(int64_t utime) {
  return ros::Time(uint32_t(utime/1E6), uint32_t(utime%uint64_t(1E6) * 1000));
}

struct PoseEntry
{
  PoseEntry(int64_t utime, double x, double y, double heading)
    : utime(utime), x(x), y(y), heading(heading) {}

  int64_t utime;
  double x;
  double y;
  double heading;
};

struct StampedPose
{
  StampedPose() : utime(0) {}
  StampedPose(int64_t utime, const Eigen::Isometry3d & pose)
    : utime(utime), pose(pose) {}
  int64_t utime;
  Eigen::Isometry3d pose;
};

class Trajectory
{
public:
  Trajectory() {}
  ~Trajectory() {}

  /**
   * Get pose at the given time.
   */
  StampedPose get_pose(int64_t t) {
    TrajectoryList::iterator it = poses_.lower_bound(t);
    assert( (it != poses_.end()) && "Only newer values exist. Backward extrapolation is not supported.");
    // @todo should we throw an exception?

    // Check if there is an exact match
    if ((it->second).utime == t) return it->second;

    StampedPose pose2 = it->second;

    --it;
    assert ( (it != poses_.end()) && "No newer value exists. Forward extrapolation is not supported.");

    StampedPose pose1 = it->second;

    double a = double(t-pose1.utime) / double(pose2.utime-pose1.utime);
    Eigen::Quaterniond q1 = Eigen::Quaterniond(pose1.pose.rotation());
    Eigen::Quaterniond q2 = Eigen::Quaterniond(pose2.pose.rotation());
    Eigen::Quaterniond q_t = q1.slerp(a, q2);
    Eigen::Vector3d t_t = (1.0 - a)*pose1.pose.translation() + a*pose2.pose.translation();

    // Pose at time t;
    StampedPose pose_t = pose1;
    pose_t.utime = t;
    pose_t.pose.setIdentity();
    pose_t.pose.rotate(q_t);
    pose_t.pose.translation() = t_t;

    return pose_t;
  }

  int64_t get_oldest_time() const {return poses_.begin()->second.utime;}
  int64_t get_latest_time() const {return poses_.rbegin()->second.utime;}

  bool can_interpolate(int64_t t) const { return (poses_.size() > 0 && get_oldest_time()<=t && get_latest_time()>=t);}

  void add(const StampedPose & pose) { poses_[pose.utime] = pose; }

  size_t size() {return poses_.size();}
private:
  // Stores a list of poses sorted by time.
  typedef std::map<uint64_t, StampedPose, std::less<uint64_t>, Eigen::aligned_allocator<std::pair<uint64_t, StampedPose> > > TrajectoryList;
  TrajectoryList poses_;
};

void read_file(const std::string & filename, std::vector<PoseEntry> & poses)
{
  std::fstream gt_file;
  gt_file.open(filename.c_str());
  if (!gt_file.is_open()) return ; // Nothing to read

  long long utime;
  std::string line;
  double x, y, h;
  while (!gt_file.eof())
  {
    std::getline(gt_file, line);
      int n = sscanf(line.c_str(), "%Lu,%lf,%lf,%lf", &utime, &x, &y, &h);
      if (n == 4)
      {
        poses.push_back(PoseEntry(utime, x, y, h));
      }
     cout << utime << "\n";
/*

    if (line.compare(0, 4, "POSE") == 0)
    {
      int n = sscanf(line.c_str(), "POSE %Ld %lf %lf %lf", &utime, &x, &y, &h);
      if (n == 4)
      {
        poses.push_back(PoseEntry(utime, x, y, h));
      }
    }
    else if (line.compare(0, 6, "GLOBAL") == 0)
    {
      int n = sscanf(line.c_str(), "GLOBAL,%Ld,%lf,%lf,%lf", &utime, &x, &y, &h);
      //int n = sscanf(line.c_str(), "%Lu,%lf,%lf,%lf", &utime, &x, &y, &h);
      if (n == 4)
      {
        poses.push_back(PoseEntry(utime, x, y, h));
      }
    }*/
  }
}

void read_file_new(const std::string & filename, std::vector<PoseEntry> & poses)
{
  std::fstream gt_file;
  gt_file.open(filename.c_str());
  if (!gt_file.is_open()) return ; // Nothing to read

  //uint64_t utime;
  long long utime;
  std::string line;
  double x, y, h;
  while (!gt_file.eof())
  {
    std::getline(gt_file, line);
    int n = sscanf(line.c_str(), "%Ld,%lf,%lf,%lf", &utime, &x, &y, &h);
    if (n == 4)
    {
      poses.push_back(PoseEntry(utime, x, y, h));
    }
  }
}


Eigen::Vector3d transform_to_rpy(const tf::Transform & transform)
{
  tf::Quaternion q = transform.getRotation();
  return Eigen::Isometry3d(Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()).matrix()).rotation().eulerAngles(0,1,2);
}

int main(int argc, char** argv)
{
  ConciseArgs parser(argc, argv, "Compute transform from base to camera");
  std::string log_fname = "/media/passport1/data/pr2/2012/icra/2012-01-18-09-09-07.bag";
  //std::string gt_fname = "/home/hordurj/data/groundtruth/2012-04-06-11-15-29.gt";
  std::string gt_fname = "/home/mfallon/Desktop/hordur_icra/2012-01-18-09-09-07_part1.gt.smoothed";
  std::string camera_fname = "camera.gt";
  parser.add(log_fname, "l", "log_fname", "ROS bag filename");
  parser.add(gt_fname, "g", "gt_fname", "groundtruth filename");
  parser.add(camera_fname, "c", "camera_fname", "Camera pose output filename");
  parser.parse();
  std::cout << log_fname << " is log_fname\n";
  std::cout << gt_fname << " is gt_fname\n";
  std::cout << camera_fname << " is camera_fname\n";

  // File to save out poses
  std::fstream camera_file;
  camera_file.open(camera_fname.c_str(), std::fstream::out);

  // Contains the ground truth camera trajectory
  Trajectory camera_path;

  // Camera framestaps that have yet to be written out
  std::vector<uint64_t> camera_frame_buffer;

  std::vector<PoseEntry> poses;
  read_file_new(gt_fname, poses);
  size_t poses_position = 0;

  uint64_t last_tf_utime = 0;

  ros::init(argc, argv, "compute_transforms_ros");

  ros::Time::init();
  rosbag::Bag bag;
  bag.open(log_fname , rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back("/tf");

  //std::string camera_topic = "/camera/rgb/camera_info";
  //std::string camera_frame = "/openni_rgb_frame";

  std::string camera_topic = "/wide_stereo/left/camera_info";
  std::string camera_frame = "/wide_stereo_link";
  topics.push_back(camera_topic);

  tf::Transformer transformer;

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    if (m.getTopic() == camera_topic)
    {
        sensor_msgs::CameraInfoPtr info = m.instantiate<sensor_msgs::CameraInfo>();

        uint64_t utime = stamp_to_utime(info->header.stamp);
        camera_frame_buffer.push_back(utime);

        int end_pos = 0;
        for (size_t i = 0; i < camera_frame_buffer.size(); ++i)
        {
          if (camera_path.can_interpolate(camera_frame_buffer[i]))
          {
            //isam::Pose3d

            StampedPose pose = camera_path.get_pose(camera_frame_buffer[i]);
            std::cout << "pose: " << pose.utime << " " << pose.pose.translation().x() << std::endl;
            Eigen::Quaterniond q(pose.pose.rotation());
            camera_file << pose.utime << ","
                        << pose.pose.translation().x() << ","
                        << pose.pose.translation().y() << ","
                        << pose.pose.translation().z() << ","
                        << q.x() << ","
                        << q.y() << ","
                        << q.z() << ","
                        << q.w() << std::endl;
            end_pos = i;

            Eigen::Vector3d v = Eigen::Isometry3d(q).rotation().eulerAngles(0,1,2);
            //std::cout << "rpy: " << v(0)/M_PI*180.0 << " " << v(1)/M_PI*180.0 << " " << v(2)/M_PI*180.0 << std::endl;
          }
          else if (end_pos != 0)
          {
            break ;
          }
        }
        if (end_pos != 0) camera_frame_buffer.erase(camera_frame_buffer.begin(), camera_frame_buffer.begin() + end_pos + 1);
    }
    else if (m.getTopic() == "/wide_stereo/left/camera_info")
    {
      sensor_msgs::CameraInfoPtr info = m.instantiate<sensor_msgs::CameraInfo>();

        uint64_t utime = stamp_to_utime(info->header.stamp);
        camera_frame_buffer.push_back(utime);

        int end_pos = 0;
        for (size_t i = 0; i < camera_frame_buffer.size(); ++i)
        {
          if (camera_path.can_interpolate(camera_frame_buffer[i]))
          {
            //isam::Pose3d

            StampedPose pose = camera_path.get_pose(camera_frame_buffer[i]);
            std::cout << "pose: " << pose.utime << " " << pose.pose.translation().x() << std::endl;
            Eigen::Quaterniond q(pose.pose.rotation());
            camera_file << pose.utime << ","
                        << pose.pose.translation().x() << ","
                        << pose.pose.translation().y() << ","
                        << pose.pose.translation().z() << ","
                        << q.x() << ","
                        << q.y() << ","
                        << q.z() << ","
                        << q.w() << std::endl;
            end_pos = i;
  
            Eigen::Vector3d v = Eigen::Isometry3d(q).rotation().eulerAngles(0,1,2);
            //std::cout << "rpy: " << v(0)/M_PI*180.0 << " " << v(1)/M_PI*180.0 << " " << v(2)/M_PI*180.0 << std::endl;
          }
          else if (end_pos != 0)
          {
            break ;
          }
        }
        if (end_pos != 0) camera_frame_buffer.erase(camera_frame_buffer.begin(), camera_frame_buffer.begin() + end_pos + 1);

      //uint64_t utime = stamp_to_utime(info->header.stamp);
      // Lookup
      //std::cout << "Lookup camera path at " << utime << std::endl;
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
          last_tf_utime = stamp_to_utime(transform.stamp_);
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
        std::cout << poses_position << " " << poses.size() << std::endl;
        if (poses_position < poses.size())
        {
          PoseEntry pose_entry = poses[poses_position];
          uint64_t current_time = pose_entry.utime;
          ros::Time stamp = utime_to_stamp(current_time);

          std::cout << "time: " << current_time << " " << last_tf_utime <<
               "  " << (last_tf_utime - current_time) <<
               "  " << (static_cast<double>(last_tf_utime) - static_cast<double>(current_time))*1e-6 <<
               "  " << poses_position << std::endl;

          if (transformer.canTransform("/base_footprint", "/base_link", stamp))
          {
              tf::StampedTransform transform;
              transformer.lookupTransform("/base_footprint", "/base_link", stamp, transform);
              tf::Vector3 t = transform.getOrigin();
              tf::Quaternion q = transform.getRotation();
              Eigen::Vector3d v = Eigen::Isometry3d(Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z())).rotation().eulerAngles(0,1,2);
              std::cout << "footprint->base rpy: " << v(0)/M_PI*180.0 << " " << v(1)/M_PI*180.0 << " " << v(2)/M_PI*180.0 << std::endl;
          }

          if (transformer.canTransform("/base_link", "/base_laser_link", stamp))
          {
              tf::StampedTransform transform;
              transformer.lookupTransform("/base_link", "/base_laser_link", stamp, transform);
              tf::Vector3 t = transform.getOrigin();
              tf::Quaternion q = transform.getRotation();
              Eigen::Vector3d v = Eigen::Isometry3d(Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z())).rotation().eulerAngles(0,1,2);
              std::cout << "base->laser rpy: " << v(0)/M_PI*180.0 << " " << v(1)/M_PI*180.0 << " " << v(2)/M_PI*180.0 << std::endl;
          }

          if (transformer.canTransform("/base_link", camera_frame, stamp))
          {
              tf::StampedTransform transform;
              transformer.lookupTransform("/base_link", camera_frame, stamp, transform);
              tf::Vector3 t = transform.getOrigin();
              tf::Quaternion q = transform.getRotation();
              Eigen::Vector3d v = Eigen::Isometry3d(Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z())).rotation().eulerAngles(0,1,2);
              std::cout << "base->camera rpy: " << v(0)/M_PI*180.0 << " " << v(1)/M_PI*180.0 << " " << v(2)/M_PI*180.0 << std::endl;
          }

          if (    transformer.canTransform("/odom_combined", camera_frame, stamp)
               && transformer.canTransform("/base_laser_link", camera_frame, stamp))
          {
            poses_position++;

            tf::StampedTransform odom_transform;
            transformer.lookupTransform("/odom_combined", camera_frame, stamp, odom_transform);
            Eigen::Vector3d rpy = transform_to_rpy(odom_transform);

            tf::StampedTransform transform;
            transformer.lookupTransform("/base_laser_link", camera_frame, stamp, transform);
            tf::Vector3 t = transform.getOrigin();
            tf::Quaternion q = transform.getRotation();
            if (VERBOSE) std::cout << "/base_laser_link -> " << camera_frame << " " << t.x() << " " << t.y() << " " << t.z() << std::endl;

            Eigen::Isometry3d gt_laser;      // Groundtruth pose for laser
            Eigen::Isometry3d laser_camera;  // Laser to camera transformation

            /*
            gt_laser = Eigen::Isometry3d(
                  Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX())
                * Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(pose_entry.heading, Eigen::Vector3d::UnitZ()) );
            */
            gt_laser = Eigen::Isometry3d(
                  Eigen::AngleAxisd(pose_entry.heading, Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX()) );
            gt_laser.translation() = Eigen::Vector3d(pose_entry.x, pose_entry.y, 0.0);

            tf::Vector3 axis = q.getAxis();
            laser_camera = Eigen::Isometry3d(Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()).matrix());
            laser_camera.translation() = Eigen::Vector3d(t.x(), t.y(), t.z());

            {
            Eigen::Vector3d v = Eigen::Isometry3d(Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z())).rotation().eulerAngles(0,1,2);
            std::cout << "laser rpy: " << v(0)/M_PI*180.0 << " " << v(1)/M_PI*180.0 << " " << v(2)/M_PI*180.0 << std::endl;
            }

            StampedPose pose;
            pose.utime = current_time;
            pose.pose = gt_laser * laser_camera; // Compute gt -> camera transform
            //pose.pose = gt_laser;
            camera_path.add(pose);

            //transformer.lookupTransform("/base_laser_link", "/openni_rgb_frame", ros::Time(0), transform);
            //t = transform.getOrigin();
            //if (VERBOSE) std::cout << "/base_laser_link -> /openni_rgb_frame: " << t.x() << " " << t.y() << " " << t.z() << std::endl;
          }
          else if ((static_cast<double>(last_tf_utime) - static_cast<double>(current_time))*1e-6 > 1.0)
          {
            std::cout << "Skipping frame" << std::endl;
            poses_position++;
          }
        }
      }
      catch (tf::TransformException ex)
      {
         ROS_ERROR("lookupTransform failed: %s", ex.what());
      }
    }
  }

  camera_file.close();

  return 0;
}
