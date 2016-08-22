#ifndef __ALIGN_UTILS_H__
#define __ALIGN_UTILS_H__

#include "align_utils.hpp"
#include <algorithm>

using namespace std;
using namespace isam;
using namespace Eigen;
using namespace scanmatch;


#define SHOW_GUI 1
#define LARGE_CLOUD_SKIP 20

void AlignUtils::load_config(){
}

struct IncrPrior
{
  IncrPrior(Pose2d incr, Pose2d prior) : incr(incr), prior(prior) {}
  Pose2d incr;
  Pose2d prior;
};

std::ostream& operator<<(std::ostream& out, const IncrPrior& p) {
  p.incr.write(out);
  out << ",";
  p.prior.write(out);
  return out;
}

/**
 * Prior on Pose2d.
 */
class Pose2d_Incr_Factor : public FactorT<IncrPrior> {
  Pose2d_Node* _pose;
  Pose2d _incr;
  Pose2d _prior;

public:

  /**
   * Constructor.
   * @param pose The pose node the prior acts on.
   * @param prior The actual prior measurement.
   * @param noise The 3x3 square root information matrix (upper triangular).
   */
  Pose2d_Incr_Factor(Pose2d_Node* pose, const Pose2d& incr, const Pose2d& prior, const Noise& noise)
    : FactorT<IncrPrior >("Pose2d_Incr_Factor", 3, noise, IncrPrior(incr, prior)), _pose(pose), _incr(incr), _prior(prior) {
    _nodes.resize(1);
    _nodes[0] = pose;
  }

  void initialize() {
    //if (!_pose->initialized()) {
    //  Pose2d predict = _measure;
    //  _pose->init(predict);
    //}
  }

  Eigen::VectorXd basic_error(Selector s = LINPOINT) const {
    Pose2d pose = Pose2d(_pose->vector(s));
    Pose2d origin;
    Pose2d delta = origin.ominus(pose.ominus(_prior));
    Eigen::VectorXd err =   Eigen::Vector3d(delta.x(), delta.y(), delta.t())
                          - Eigen::Vector3d(_incr.x(), _incr.y(), _incr.t());
    err(2) = standardRad(err(2));
    return err;
  }
};


template<class T>
static void delete_item(T* item) { delete item; }

void AlignUtils::smooth_poses(isam::Pose2d first_pose,
                                  isam::Pose2d last_pose,
                                  const std::vector<isam::Pose2d> & incremental_poses,
                                  std::vector<isam::Pose2d> * smoothed_poses)
{
  Slam slam;
  Properties prop;
  prop.max_iterations = 100;
  prop.method = isam::DOG_LEG;

  slam.set_properties(prop);
  std::vector<Pose2d_Node*> nodes;
  std::vector<Factor*> factors;

  Matrix3d noise_matrix = 100. * Matrix<double, 3, 3>::Identity();
  //noise_matrix(0,0) =0.01; // weaken the x position alignment constraint, relative to theta
  //noise_matrix(1,1) =0.01; // weaken the y position alignment constraint, relative to theta
  Noise noise3 = Information(noise_matrix);


  size_t n = incremental_poses.size();
  size_t i = 0;

  // create a prior measurement (a factor)
  Pose2d origin = first_pose.oplus(incremental_poses[i]);
  nodes.push_back(new Pose2d_Node());
  Pose2d_Factor* prior = new Pose2d_Factor(nodes[0], origin, noise3);
  factors.push_back(prior);
  slam.add_factor(prior);
  ++i;

  while (i < n-1)
  {
    nodes.push_back(new Pose2d_Node());
    slam.add_node(nodes.back());

    // connect to previous with odometry measurement
    Pose2d_Pose2d_Factor* constraint = new Pose2d_Pose2d_Factor(nodes[i-1], nodes[i], incremental_poses[i], noise3);
    slam.add_factor(constraint);

    ++i;
  }

  Pose2d_Incr_Factor* end_factor = new Pose2d_Incr_Factor(nodes.back(), incremental_poses[i], last_pose, noise3);
  slam.add_factor(end_factor);

  slam.batch_optimization();

  for (std::vector<isam::Pose2d_Node*>::iterator it = nodes.begin(); it != nodes.end() ;++it)
  {
    smoothed_poses->push_back((*it)->value());
  }

  // Cleanup
  std::for_each(factors.begin(), factors.end(), delete_item<Factor>);
  std::for_each(nodes.begin(), nodes.end(), delete_item<Pose2d_Node>);
}



Eigen::Quaterniond euler_to_quat_align_utils(double yaw, double pitch, double roll) {
  double sy = sin(yaw*0.5);
  double cy = cos(yaw*0.5);
  double sp = sin(pitch*0.5);
  double cp = cos(pitch*0.5);
  double sr = sin(roll*0.5);
  double cr = cos(roll*0.5);
  double w = cr*cp*cy + sr*sp*sy;
  double x = sr*cp*cy - cr*sp*sy;
  double y = cr*sp*cy + sr*cp*sy;
  double z = cr*cp*sy - sr*sp*cy;
  return Eigen::Quaterniond(w,x,y,z);
}

void quat_to_euler_align_utils(Eigen::Quaterniond q, double& yaw, double& pitch, double& roll) {
  const double q0 = q.w();
  const double q1 = q.x();
  const double q2 = q.y();
  const double q3 = q.z();
  roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
  pitch = asin(2*(q0*q2-q3*q1));
  yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
}



void AlignUtils::read_log(std::string poses_files,
      std::vector<SmoothBlock>& blocks,
      float &rad0, float &radstep ){

  int counter=0;
  string line;
  ifstream myfile (poses_files.c_str());
  if (myfile.is_open()){

    getline (myfile,line);
    int64_t global_utime;
    double x, y, t;
    if (line.compare(0,6,"GLOBAL") == 0){
      int res = sscanf(line.c_str(), "GLOBAL,%ld,%lf,%lf,%lf", &global_utime, &x, &y, &t);
    }else{
      cout << "First line in file must lead off with GLOBAL\n";
      exit(-1);
    }
    isam::Pose2d global_Pose(x,y,t);


    vector< isam::Pose2d > incr_Poses;
    vector< int64_t> incr_Preutime;
    vector< int64_t> incr_Postutime;
    vector< vector <float> > scan_stack;

    int64_t Preutime;
    int64_t Postutime = 0;

    counter =0;
    while ( myfile.good() ){

      getline (myfile,line);
      counter++;
      if (line.size() > 4){
        if (line.compare(0,11,"INCREMENTAL") == 0){
          //cout << "<incr>\n";
          double x, y, t;
          int res = sscanf(line.c_str(), "INCREMENTAL,%ld,%ld,%lf,%lf,%lf", &Preutime,&Postutime, &x, &y, &t);
          isam::Pose2d incr_Pose(x,y,t);
          incr_Poses.push_back(incr_Pose);
          incr_Preutime.push_back(Preutime);
          incr_Postutime.push_back(Postutime);
        }else if (line.compare(0,4,"SCAN") == 0) {
          //cout << "<scan>\n";
          //cout << line << " is the scan\n";
          //int pause;
          //cin >> pause;

          char_separator<char> sep(",");
          tokenizer< char_separator<char> > tokens(line, sep);
          int count=0;
          vector <float> scan;
          int expected_scan_size=0;
          BOOST_FOREACH (const string& t, tokens) {
            //cout << t << "." << endl;
            if (count==0){
              // "SCAN"
            }else if (count==1){
              int64_t scan_utime = atol ( t.c_str() );
              //cout << scan_utime << " scan_utime\n";
              //cout << Postutime << " Postutime\n";
              if (scan_utime!=Postutime){
                cout << "ERROR: order of data should be pose scan pose scan\n";
                exit(-1);
              }
            }else if(count == 2){
              rad0 = atof ( t.c_str() );
              //cout << rad0 << " rad0\n";
            }else if(count==3){
              radstep = atof ( t.c_str() );
              //cout << radstep << " radstep\n";
            }else if(count==4){
              expected_scan_size = atoi ( t.c_str() );
              //cout << expected_scan_size << " count\n";
            }else{
              float val = atof ( t.c_str() );
              scan.push_back(val);
            }
            count++;
          }
          //cout << scan.size() << " is the scan size\n";
          if (scan.size() != expected_scan_size){
            cout << "ERROR: " << line << "\n";
            cout << "ERROR: scan size: "<< scan.size() <<"\n";
            cout << "ERROR: expected: "<< expected_scan_size <<"\n";
            cout << "ERROR: scan size should match contents\n";
            exit(-1);
          }
          scan_stack.push_back(scan);
        }else if (line.compare(0,6,"GLOBAL") == 0){
          int res = sscanf(line.c_str(), "GLOBAL,%ld,%lf,%lf,%lf", &global_utime, &x, &y, &t);
          isam::Pose2d next_global_Pose = isam::Pose2d(x,y,t);

          // Pass to output:
          SmoothBlock block;
          block.first_pose = global_Pose;
          block.last_pose = next_global_Pose;
          block.last_utime = global_utime;
          block.last_scan = scan_stack[ scan_stack.size()-1 ]  ;

          for (size_t i=0; i<scan_stack.size(); i++){
            block.scans.push_back(scan_stack[i]);
          }
          for (size_t i=0; i<incr_Poses.size(); i++){
            block.incremental_poses.push_back(incr_Poses[i]);
            block.incremental_preutime.push_back(incr_Preutime[i]);
          }
          blocks.push_back(block);

          //cout << "BLOCK: scans " << scan_stack.size() << " | " << incr_Poses.size() << "\n";
          //cout << "blocks: " << blocks.size() << "\n";
          //cout << "counter: " << counter << "\n";

          // Reset and go again
          global_Pose = next_global_Pose;
          incr_Poses.clear();
          incr_Preutime.clear();
          incr_Postutime.clear();
          scan_stack.clear();
        }else{
          cout << line << "\n";
          cout << "not matched...\n";
          exit(-1);
        }
      }else{
        cout << "[ "<< line << "] ... short\n";
        //exit(-1);
      }
    }
    myfile.close();
  } else{
    printf( "Unable to open poses file\n%s",poses_files.c_str());
    return;
  }

  cout << "read " << counter << " lines\n";
}


void AlignUtils::smooth_log(vector < SmoothBlock >& blocks){

  for (size_t i = 0; i < blocks.size(); i++) {

    std::vector<isam::Pose2d> smoothed_poses;
    smooth_poses(blocks[i].first_pose, blocks[i].last_pose, blocks[i].incremental_poses, &smoothed_poses);
    blocks[i].smoothed_poses.clear();
    for (size_t j = 0; j < smoothed_poses.size(); j++){
      Pose2d pose = smoothed_poses[j];
      blocks[i].smoothed_poses.push_back(pose);
    }
    blocks[i].smoothed_poses.push_back(blocks[i].last_pose);

  }

}


void AlignUtils::convert_smoothed_log_to_output(std::vector<isam::Pose2d>& poses,
    std::vector< int64_t>& pose_utimes,
    vector < vector <float> > &scans, vector < SmoothBlock >& blocks, int do_smooth ){

  SmoothBlock first_block = blocks[ 0 ];
  poses.push_back(first_block.first_pose);
  vector < float> empty_scan;
  scans.push_back(empty_scan); // this scan is not retained

  for (size_t i = 0; i < blocks.size(); i++) {
    for (size_t j=0; j<blocks[i].scans.size(); j++){
      scans.push_back( blocks[i].scans[j] );
    }
    if (do_smooth){
      for (size_t j = 0; j < blocks[i].smoothed_poses.size(); j++){
        Pose2d pose = blocks[i].smoothed_poses[j];
        poses.push_back(pose);
      }
      for (size_t j=0; j< blocks[i].incremental_preutime.size(); j++){
        pose_utimes.push_back( blocks[i].incremental_preutime[j]);
      }
    }else{
      // This is where incremental poses are converted to global poses:
      vector< isam::Pose2d > global_stack;
      global_stack = makeProblemGlobal(blocks[i].first_pose,
          blocks[i].incremental_poses);

      // NB: Skip the 1st pose as its scan is in the last batch of measurements
      for (size_t j=1; j<global_stack.size(); j++){
        poses.push_back(global_stack[j]);
      }
      for (size_t j=0; j< blocks[i].incremental_preutime.size(); j++){
        pose_utimes.push_back( blocks[i].incremental_preutime[j]);
      }
    }
  }
  // Add in the final pose_utimes
  SmoothBlock last_block = blocks[ blocks.size()-1 ];
  pose_utimes.push_back(last_block.last_utime);

  //cout << scans.size() << " scans and\n";
  //cout << poses.size() << " poses transformed [and smoothed]q\n";
  //cout << pose_utimes.size() << " pose_utimes\n";
}

// Paired with the function below:
void AlignUtils::write_smoothed_log(std::string poses_files,std::vector<isam::Pose2d>& poses,
    std::vector< int64_t >& pose_utimes,
    vector < vector <float> > &scans,std::string file_extension){

  string smoothed_filename = poses_files + "." + file_extension;
  std::fstream smoothed_file;
  smoothed_file.open(smoothed_filename.c_str(), std::fstream::out);
  for (size_t i = 0; i < poses.size(); i++) {
    isam::Pose2d pose = poses[i];
    smoothed_file << pose_utimes[i] << "," << pose.x() << "," << pose.y() << "," << pose.t() << std::endl;
  }
  //cout << scans.size() << " scans and\n";
  cout << poses.size() << " poses written\n";
  cout << pose_utimes.size() << " pose_utimes written\n";
  smoothed_file.close();
}


// Paired with the function above:
void AlignUtils::write_smoothed_log_scans(std::string poses_files,std::vector<isam::Pose2d>& poses,
    std::vector< int64_t >& pose_utimes,
    vector < vector <float> > &scans,std::string file_extension,
    float &rad0, float &radstep ){

  string smoothed_filename = poses_files + "." + file_extension;
  std::fstream smoothed_file;
  smoothed_file.open(smoothed_filename.c_str(), std::fstream::out);
  for (size_t i = 0; i < scans.size(); i++) {
    int nranges = scans[i].size();
    ostringstream temp2;
    temp2 << pose_utimes[i] << ","
        << rad0 << ","
        << radstep << ","
        << nranges ;
    for (size_t j=0;j < scans[i].size(); j++){
      temp2 << "," << scans[i][j];
    }
    smoothed_file << temp2.str() << endl;
  }

  //cout << scans.size() << " scans and\n";
  cout << scans.size() << " scans written using\n";
  cout << pose_utimes.size() << " pose_utimes written\n";
  smoothed_file.close();
}


void AlignUtils::read_poses_and_scans(std::string poses_files, std::vector<isam::Pose2d>& poses,
    vector < vector <float> > &scans, float &rad0, float &radstep ){
  int counter=0;
  string line;
  ifstream myfile (poses_files.c_str());
  int64_t pose_utime =0;
  if (myfile.is_open()){
    while ( myfile.good() ){
      getline (myfile,line);

      if (line.size() > 4){
        //cout << line << " was read\n";
        char keyword_c[20000];
        int key_length;
        sscanf(line.c_str(), "%s%n", keyword_c, &key_length);
        //const char* arguments = &str[key_length];
        string keyword(keyword_c);
        //cout << keyword << " is keyword\n";

        if (keyword == "POSE") {
          string line_end = line.substr (5);
          //cout << line_end << "| is the end\n";
          int64_t utime;
          float x, y, t;
          int res = sscanf(line_end.c_str(), "%ld %f %f %f", &utime, &x, &y, &t);
          isam::Pose2d newPose(x,y,t);
          poses.push_back(newPose);
          //cout << x << " and " << y <<"\n";
          pose_utime= utime;
        }else if (keyword == "SCAN") {
          string line_end = line.substr (5);
          //cout << line_end << "| is the end\n";

          char_separator<char> sep(", ");
          tokenizer< char_separator<char> > tokens(line_end, sep);
          int count=0;
          vector <float> scan;
          int expected_scan_size=0;
          BOOST_FOREACH (const string& t, tokens) {
            //cout << t << "." << endl;
            if (count==0){
              int64_t scan_utime = atol ( t.c_str() );
              //cout << scan_utime << " scan_utime\n";
              //cout << pose_utime << " pose_utime\n";
              if (scan_utime!=pose_utime){
                cout << "ERROR: order of data should be pose scan pose scan\n";
                exit(-1);
              }
            }else if(count==1){
              rad0 = atof ( t.c_str() );
              //cout << rad0 << " rad0\n";
            }else if(count==2){
              radstep = atof ( t.c_str() );
              //cout << radstep << " radstep\n";
            }else if(count==3){
              expected_scan_size = atoi ( t.c_str() );
              //cout << expected_scan_size << " count\n";
            }else{
              float val = atof ( t.c_str() );
              scan.push_back(val);
            }
            count++;
          }
          //cout << scan.size() << " is the scan size\n";
          if (scan.size() != expected_scan_size){
            cout << "ERROR: " << line_end << "\n";
            cout << "ERROR: scan size: "<< scan.size() <<"\n";
            cout << "ERROR: exprected: "<< expected_scan_size <<"\n";
            cout << "ERROR: scan size should match contents\n";
            exit(-1);
          }
          scans.push_back(scan);
        }
      }
    }
    myfile.close();
  } else{
    printf( "Unable to open poses file\n%s",poses_files.c_str());
    return;
  }    
  cout << scans.size() << " scans captured\n";
}



void AlignUtils::sendPoseIsometry3d(Eigen::Isometry3d &pose,int pose_coll,
			  int64_t pose_id, std::string pose_name, bool reset)
{
  vs_obj_collection_t objs;	
  size_t n =1;// collection.size();
  //	if (n > 1) {
  objs.id = pose_coll;//collection.id();
  objs.name = (char*) pose_name.c_str();
  objs.type = VS_OBJ_COLLECTION_T_AXIS3D;//VS_OBJ_COLLECTION_T_POSE;
  objs.reset = reset; //reset;
  objs.nobjs = n;
  vs_obj_t poses[n];
  for (size_t i = 0; i < n; i++) {
    poses[i].id = pose_id;// collection(i).utime;

  Eigen::Vector3d t(pose.translation());
//  Eigen::Quaterniond r(pose.rotation());
 // ss <<t[0]<<", "<<t[1]<<", "<<t[2]<<" | " 
   //    <<r.w()<<", "<<r.x()<<", "<<r.y()<<", "<<r.z() ;
  //  std::cout << ss.str() << "q\n";

    poses[i].x = t[0];
    poses[i].y = t[1];
    poses[i].z = t[2];

/*    poses[i].x = pose.translation.x() ;
    poses[i].y = pose.translation.y() ;
    poses[i].z =pose.translation.z();*/


  // Eigen::Vector3d rpy = pose.rotation().eulerAngles(0,1,2);
   // poses[i].roll =rpy(0);
   // poses[i].pitch =rpy(1);
   // poses[i].yaw = rpy(2);

//    Eigen::Quaterniond r(pose.rotation());
  //  quat_to_euler_align_utils(r, poses[i].yaw, poses[i].pitch, poses[i].roll) ;

    Eigen::Quaterniond q(pose.rotation());
    Eigen::Vector3d rpy = Eigen::Isometry3d(Eigen::Quaterniond(q.w(), q.x(),
    q.y(), q.z())).rotation().eulerAngles(0,1,2);
    poses[i].yaw = rpy[2];
    poses[i].pitch = rpy[1];
    poses[i].roll = rpy[0];


  }
  objs.objs = poses;
  vs_obj_collection_t_publish(lcmref_, "OBJ_COLLECTION", &objs);
  //	}
}




void AlignUtils::sendPose(isam::Pose2d pose,int pose_coll,
			  int64_t pose_id, std::string pose_name, bool reset)
{
  vs_obj_collection_t objs;	
  size_t n =1;// collection.size();
  //	if (n > 1) {
  objs.id = pose_coll;//collection.id();
  objs.name = (char*) pose_name.c_str();
  objs.type = VS_OBJ_COLLECTION_T_AXIS3D;//VS_OBJ_COLLECTION_T_POSE;
  objs.reset = reset; //reset;
  objs.nobjs = n;
  vs_obj_t poses[n];
  for (size_t i = 0; i < n; i++) {
    poses[i].id = pose_id;// collection(i).utime;
    poses[i].x = pose.x() ;
    poses[i].y = pose.y() ;
    poses[i].z =0; //pose.z();
    poses[i].yaw = pose.t();
    poses[i].pitch =0;// pose.pitch();
    poses[i].roll =0;// pose.roll();
  }
  objs.objs = poses;
  vs_obj_collection_t_publish(lcmref_, "OBJ_COLLECTION", &objs);
  //	}
}


void AlignUtils::sendPoseIsometry3dVector(vector< Eigen::Isometry3d > poseVector,
    int pose_coll,int64_t pose_id, std::string pose_name, int type)
{
  vs_obj_collection_t objs;
  size_t n =poseVector.size();// collection.size();
  //  if (n > 1) {
  objs.id = pose_coll;//collection.id();
  objs.name = (char*) pose_name.c_str();
  objs.type = type;//;VS_OBJ_COLLECTION_T_POSE;
  objs.reset = false; //reset;
  objs.nobjs = n;
  vs_obj_t poses[n];
  for (size_t i = 0; i < n; i++) {
    poses[i].id   = pose_id+i;// collection(i).utime;
    Eigen::Vector3d t(poseVector[i].translation());
    poses[i].x = t[0];
    poses[i].y = t[1];
    poses[i].z = t[2];

    Eigen::Quaterniond q(poseVector[i].rotation());
    Eigen::Vector3d rpy = Eigen::Isometry3d(Eigen::Quaterniond(q.w(), q.x(),
    q.y(), q.z())).rotation().eulerAngles(0,1,2);
    poses[i].yaw = rpy[2];
    poses[i].pitch = rpy[1];
    poses[i].roll = rpy[0];

    //Eigen::Quaterniond r(poseVector[i].rotation());
    //quat_to_euler_align_utils(r, poses[i].yaw, poses[i].pitch, poses[i].roll) ;
  }
  objs.objs = poses;
  vs_obj_collection_t_publish(lcmref_, "OBJ_COLLECTION", &objs);
}





void AlignUtils::sendPoseVector(vector< isam::Pose2d > poseVector,int pose_coll,
    int64_t pose_id, std::string pose_name, int type)
{
  int incr =1;
  if(poseVector.size()>10000){
    //cout << "large posevector\n";
    incr=LARGE_CLOUD_SKIP;
  }


  vs_obj_collection_t objs;	
  size_t n =poseVector.size();// collection.size();
  //	if (n > 1) {
  objs.id = pose_coll;//collection.id();
  objs.name = (char*) pose_name.c_str();
  objs.type = type;
  objs.reset = false; //reset;
  objs.nobjs = n;
  vs_obj_t poses[n];

  int counter =0;
  for (size_t i = 0; i < n; i=i+incr) {
    poses[counter].id   = pose_id+i;// collection(i).utime;
    poses[counter].x    = poseVector[i].x() ;
    poses[counter].y    = poseVector[i].y() ;
    poses[counter].z    = 0; //pose.z();
    poses[counter].yaw  = poseVector[i].t();
    poses[counter].pitch= 0;// pose.pitch();
    poses[counter].roll = 0;// pose.roll();
    counter++;
  }
  objs.objs = poses;
  objs.nobjs= counter;
  vs_obj_collection_t_publish(lcmref_, "OBJ_COLLECTION", &objs);
}

void AlignUtils::sendScan( PointCloudPtr & currentPointCloud,
  int pose_coll,int64_t pose_id, int scan_coll, std::string scan_name,
  bool reset)
  {

  vs_point3d_list_collection_t point_lists;

  //	std::queue<Scan*> & scans = scanDisplayQueue[mapEstimate];
  //	size_t m = collection.size();
  int m=1;

  point_lists.id = scan_coll;//collection.id();
  point_lists.name =(char *)  scan_name.c_str(); // Use channel name?
  point_lists.type = VS_POINT3D_LIST_COLLECTION_T_POINT;

  /// @todo reset if reset is requested also reset on first run
  point_lists.reset =reset;// reset;
  point_lists.nlists = m;
  vs_point3d_list_t point_list[m];

  for(size_t i=0; i<m; i++)
  {
    PointCloudPtr pointCloud = currentPointCloud;
		    
    vs_point3d_list_t* points = &(point_list[i]);
    int64_t scantime =   pose_id  ;//dataInstance->node->utime;
    points->ncolors = 0;
    points->colors = NULL;
    points->nnormals = 0;
    points->normals = NULL;
    points->npointids = 0;
    points->pointids = NULL;

    if (pointCloud) {
      size_t k = pointCloud->size();
      vs_point3d_t* entries = new vs_point3d_t[k];

      points->id = scantime;
      points->collection =  pose_coll;//collection.objectCollectionId();
      points->element_id = scantime;
      points->npoints = k;
      for (size_t j=0;j<k;j++) {
	      entries[j].x = pointCloud->points()[j].x();
	      entries[j].y = pointCloud->points()[j].y();
	      entries[j].z = pointCloud->points()[j].z();			
      }
      points->points = entries;			
    }
    else		  
    {
      points->id = 0;
      points->collection = 0;
      points->element_id = 0;
      points->npoints = 0;		  
      points->points = 0;		  
    }
  }

  point_lists.point_lists = point_list;
  vs_point3d_list_collection_t_publish(lcmref_,"POINTS_COLLECTION",&point_lists);		    
  for (int i=0;i<point_lists.nlists;i++) {
	  delete point_lists.point_lists[i].points;
  }	
}


void AlignUtils::sendScanVector( vector< PointCloudPtr > & PointClouds,
  int pose_coll,int64_t pose_id, int scan_coll, std::string scan_name,
  bool reset)
  {

/*
  int n_clouds=PointClouds.size();
  int n_clouds_max = 1000;

  int i=0;
  while (i < n_clouds){
    int n_chunk= n_clouds - i;
    n_chunk = min(n_chunk,n_clouds_max);

    vs_point3d_list_collection_t point_lists;
    point_lists.id = scan_coll;
    point_lists.name =(char *)  scan_name.c_str(); // Use channel name?
    point_lists.type = VS_POINT3D_LIST_COLLECTION_T_POINT;
    point_lists.reset =false;// ignoring reset!  reset;
    point_lists.nlists = n_chunk;
    vs_point3d_list_t point_list[n_chunk];

    for(size_t j=0; j<n_chunk; j++){
      PointCloudPtr pointCloud = PointClouds[i];
      vs_point3d_list_t* points = &(point_list[j]);
      int64_t scantime =   pose_id+i;
      points->ncolors = 0;
      points->colors = NULL;
      points->nnormals = 0;
      points->normals = NULL;
      points->npointids = 0;
      points->pointids = NULL;

      if (pointCloud) {
        size_t kkk = pointCloud->size();
        vs_point3d_t* entries = new vs_point3d_t[kkk];
        points->id = scantime;
        points->collection =  pose_coll;
        points->element_id = scantime;
        points->npoints = kkk;
        for (size_t k=0;k<kkk;k++) {
          entries[k].x = pointCloud->points()[k].x();
          entries[k].y = pointCloud->points()[k].y();
          entries[k].z = pointCloud->points()[k].z();
        }
        points->points = entries;
      }else{
        points->id = 0;
        points->collection = 0;
        points->element_id = 0;
        points->npoints = 0;
        points->points = 0;
      }

      i++;
    }
    point_lists.point_lists = point_list;
    vs_point3d_list_collection_t_publish(lcmref_,"POINTS_COLLECTION",&point_lists);
    for (int i=0;i<point_lists.nlists;i++) {
      delete point_lists.point_lists[i].points;
    }

    if (n_clouds > n_clouds_max){
      cout << "big wait for big cloud...\n";
      int pause;
      cin >> pause;

      struct timespec t;
      t.tv_sec = 1; // mfallon: was 0
      t.tv_nsec = (time_t)(50000000.5 * 1E9);  // was 0.5
      nanosleep(&t, NULL);
    }
  }
*/

  int incr =1;
  if(PointClouds.size()>10000){
    //cout << "large posevector\n";
    incr=LARGE_CLOUD_SKIP;
  }



  int m=PointClouds.size();
  vs_point3d_list_collection_t point_lists;
  point_lists.id = scan_coll;
  point_lists.name =(char *)  scan_name.c_str(); // Use channel name?
  point_lists.type = VS_POINT3D_LIST_COLLECTION_T_POINT;
  point_lists.reset =reset;
  point_lists.nlists = m;
  vs_point3d_list_t point_list[m];

  int counter =0;
  for(size_t i=0; i<m; i=i+incr){
    PointCloudPtr pointCloud = PointClouds[i];
    vs_point3d_list_t* points = &(point_list[counter]);
    int64_t scantime =   pose_id+i;
    points->ncolors = 0;
    points->colors = NULL;
    points->nnormals = 0;
    points->normals = NULL;
    points->npointids = 0;
    points->pointids = NULL;

    if (pointCloud) {
      size_t k = pointCloud->size();
      vs_point3d_t* entries = new vs_point3d_t[k];
      points->id = scantime;
      points->collection =  pose_coll;
      points->element_id = scantime;
      points->npoints = k;
      for (size_t j=0;j<k;j++) {
        entries[j].x = pointCloud->points()[j].x();
        entries[j].y = pointCloud->points()[j].y();
        entries[j].z = pointCloud->points()[j].z();
      }
      points->points = entries;
    }else{
      points->id = 0;
      points->collection = 0;
      points->element_id = 0;
      points->npoints = 0;
      points->points = 0;
    }
    counter++;
  }
  point_lists.point_lists = point_list;
  point_lists.nlists = counter;
  vs_point3d_list_collection_t_publish(lcmref_,"POINTS_COLLECTION",&point_lists);
  for (int i=0;i<point_lists.nlists;i++) {
    delete point_lists.point_lists[i].points;
  }

}


void AlignUtils::sendPoseIdVector(int n_ids, int pose_coll, int64_t pose_id, int text_coll, std::string ids_name){
  vs_text_collection_t txt_coll;
  txt_coll.id = 262;
  txt_coll.name = (char*) ids_name.c_str();
  txt_coll.type= 0;
  txt_coll.reset =true;
  txt_coll.n =n_ids;

  vs_text_t texts[txt_coll.n];
  vector<string> str_vec;
  for (int i=0;i< txt_coll.n; i++){
    vs_text_t txt;
    txt.id = pose_id+i;
    txt.collection_id = pose_coll;
    txt.object_id = pose_id+i;
    char buffer [50];
    stringstream ss;
    ss << i;
    str_vec.push_back(ss.str());
    txt.text = (char*) str_vec[i].c_str();
    texts[i] =txt;
  }
  txt_coll.texts = texts;
  vs_text_collection_t_publish(lcmref_, "TEXT_COLLECTION", &txt_coll);
}


// Take an inital global pose and compose incremental motions onto it to put poses into global frame
vector< isam::Pose2d > AlignUtils::makeProblemGlobal(  isam::Pose2d startp,
  vector< isam::Pose2d > incp){
  vector< isam::Pose2d > globp;
  isam::Pose2d previous = startp;
  globp.push_back(previous);
  for(size_t i =0; i < incp.size() ; i++){
    previous = previous.oplus( incp[i]  );
    globp.push_back(previous);
  }
  return globp;
}


//PointCloudPtr
void AlignUtils::walls2scan(PointCloudPtr &pc,std::vector<Wall> walls){
  //PointCloudPtr pc = boost::make_shared<PointCloud>();  
  
  //  cout << "eeart3\n";
  
  for (unsigned int j=0; j< walls.size(); j++){
    Wall wall;
  //      std::cout << "eeart31\n";

    wall = walls[j];   
  //  std::cout << "eeart32\n";
  
    Eigen::Vector3d sp = wall.startPos.translation();
    Eigen::Vector3d ep = wall.endPos.translation();
   
    // this should be by distance not a factor
    double dist = sqrt( pow(sp[0] - ep[0],2) + pow(sp[1] - ep[1],2) );
    
    // number of points per meter:
    double virt_res = 400;
    //was double virt_res = 40;
    double dpt = 1/(floor( dist*virt_res));
    for (double r=0; r<= 1; r=r+dpt){
      float xval = sp[0]*r + ep[0]*(1-r);
      float yval = sp[1]*r + ep[1]*(1-r);
      pc->addPoint(xval, yval, 0.0);
    //std::cout << "eeart6\n";
    }
//std::cout << "end of eeartmini\n";  
  }
//std::cout << "end of eeart\n";  
  //return pc;
}




// Create Incremental Scanmatcher
// This is NOT used by the global alignment
scanmatch::ScanMatcher* AlignUtils::initializeialignScan(){
  cout << "Initializing Incremental ScanMatcher\n";
  scanmatch::ScanMatcher* sm;

  // initial settings:
  // double motionModelPriorWeight = 0.2;
  // double addScanHitThresh = .90; 

  //hardcoded scan matcher params
  double metersPerPixel = .0075;//was .02 //translational resolution for the brute force search
  double thetaResolution = .005; //was 0.01//angular step size for the brute force search
  int useMultires = 3; // low resolution will have resolution metersPerPixel * 2^useMultiRes

  double initialSearchRangeXY = .15; //25; //nominal range that will be searched over
  double initialSearchRangeTheta = .1; //30;

  //SHOULD be set greater than the initialSearchRange
  //if a good match isn't found I'll expand and try again up to this size...
  double maxSearchRangeXY = 0.3; 
  double maxSearchRangeTheta = 0.2; 

  int maxNumScans = 15; //keep around this many scans in the history
  double addScanHitThresh = .90; //add a new scan to the map when the number of "hits" drops below this

  scanmatch::sm_incremental_matching_modes_t matchingMode;
  matchingMode = scanmatch::SM_GRID_COORD;
  //matchingMode = scanmatch::SM_COORD_ONLY;

  int useThreads = 0;
  bool verbose = true;
  bool stationaryMotionModel = true; // false;
  double motionModelPriorWeight = 0.0;
  scanmatch::ScanTransform startPose;
  memset(&startPose, 0, sizeof(startPose));

  //create the actual scan matcher object
  sm = new scanmatch::ScanMatcher(metersPerPixel, thetaResolution, useMultires, useThreads, verbose);
  sm->initSuccessiveMatchingParams(maxNumScans, initialSearchRangeXY, maxSearchRangeXY,
  initialSearchRangeTheta, maxSearchRangeTheta, matchingMode, addScanHitThresh,
  stationaryMotionModel, motionModelPriorWeight, &startPose);

  memset(&sm->currentPose, 0, sizeof(sm->currentPose));
  return sm;
}


// Incremental Scan Matching:
isam::Pose2d AlignUtils::ialignScan(PointCloudPtr & scan)
{
  // do scan matching
  int numValidPoints = scan->size();
  scanmatch::smPoint* points = new scanmatch::smPoint[numValidPoints]; 
  const std::vector<isam::Point3d> & pts = scan->points();
  for (int i=0; i<numValidPoints; i++) {
    points[i].x = pts[i].x();
    points[i].y = pts[i].y();
  }

  // Init the scanmatcher:
  if (!sm_inited){
    sm_inited = true;
    delete sm;
    sm = initializeialignScan();

    // Create a new pose when no other existed (but only used temporaily)
    lastPose = isam::Pose2d(0,0,0);
    lastPoseAdded = lastPose;
  }	
	
  isam::Pose2d new_pose(0.0, 0.0, 0.0);
  if (numValidPoints < 30) {
    // TODO: This failure is not suitable for Ground Truth - need to fix this somehow.
    std::cerr << "ODO: WARNING! NOT ENOUGH VALID POINTS! numValid=" << numValidPoints << std::endl;
    // Notify that we were not able to get a measurement between these scans 
  } else {
    scanmatch::ScanTransform r = sm->matchSuccessive(points, numValidPoints, scanmatch::SM_HOKUYO_UTM, 0, NULL);
    isam::Pose2d currentPose = isam::Pose2d(r.x, r.y, r.theta);
    lastPoseAdded = lastPose;
    lastPose = currentPose;
    
    new_pose = currentPose.ominus(lastPoseAdded);
    
    //cout << new_pose.x() << " | "
    //     << new_pose.y() << " | "
    //	   << new_pose.t() << "\n";
    //scanmatch::sm_rotateCov2D(r.sigma, -r.theta, m.cov);
    ////memcpy(m.cov, r.sigma, sizeof(m.cov));
    
    //if (SHOW_GUI)
    if (1==1)  sm->drawGUI(points, numValidPoints, r, NULL, "ialignScan");
  }

  delete [] points;
  return new_pose;
}

isam::Pose2d AlignUtils::galignScan(PointCloudPtr & walls_pc, PointCloudPtr & p1,
				    int64_t msg_utime, int nranges){
  bool useRaytracing = true; // either ray trace or use map as is. the former is better.
  
  isam::Pose2d nullPose(0,0,0);
  int null_coll =24430;
  int null_id = 123333456;
  string null_name = "Null Pose [G]";
  sendPose(nullPose,null_coll,null_id,null_name);    

  PointCloudPtr raytrace_pc = boost::make_shared<PointCloud>();
    
  
  // Ray Tracer:
  if (useRaytracing){ 
    // No necessity to use Hokuyo settings, could use more denser set of points:
    double radstep = M_PI*270.0/(nranges*180.0); // 1040 | just reduce this to make more points
    double rad0 = -3.0*M_PI/4.0;
    double maxrange = 30;
    vector < double > ranges;
    ranges.assign(nranges,maxrange);

    // initialize the points at the sensor - null range
    // this is so that points with no range are culled later
    for (size_t i=0; i< nranges; i++){
      raytrace_pc->addPoint(previousPose.x(), previousPose.y(), 0.0);
    }  
    for (size_t j=0;j < walls_pc->size();j++) { // for each point in the virtual cloud
      double dx = previousPose.x() - walls_pc->points()[j].x();
      double dy = previousPose.y() - walls_pc->points()[j].y();
      double dist = sqrt( pow(dx,2) + pow(dy,2) );
      double t = atan2(dy,dx); // CHECK THIS
      if ( dist < maxrange){ // skip if v far away
	for (int i=0; i< nranges; i++){ // for each virtual range
	  if ( dist < ranges[i]){
	    double look_t = previousPose.t() + i*radstep +   ( M_PI + rad0 );
	    double t_diff = look_t - t;
	    if (t_diff > M_PI){
	      t_diff = t_diff - M_PI*2;
	    }// TODO add other wraparound check for < -M_PI
	    if (abs( t_diff) <  (radstep/2) ){ // less than angle threshold
	      ranges[i]= dist; 
	      raytrace_pc->m_points[i] = isam::Point3d(walls_pc->points()[j].x()  , walls_pc->points()[j].y(),  0 );
	    }
	    // TODO: add break/continue on this being true... [an optimization]
	  }
	}
      }
    }
    
    sendScan(raytrace_pc,null_coll,null_id,434234, "Raytraced Walls as Points [G]" );    
   // walls_pc = raytrace_pc;
  }
  
  /// Get Alignment:
  // Subtract current pose from global point cloud:
  for (size_t i=0; i< walls_pc->size(); i++){
    walls_pc->m_points[i] = isam::Point3d(walls_pc->points()[i].x() - previousPose.x(),
				    walls_pc->points()[i].y() - previousPose.y(),
				    0 );
  }  
  for (size_t i=0; i< raytrace_pc->size(); i++){
    raytrace_pc->m_points[i] = isam::Point3d(raytrace_pc->points()[i].x() - previousPose.x(),
				    raytrace_pc->points()[i].y() - previousPose.y(),
				    0 );
  }  
  
  
  // Set zero-centered current pose to be 0,0,theta:
  isam::Pose2d previousPose_heading(0,0,previousPose.t() );
  isam::Pose2d matchPose(0,0,0);   
//  alignTwoScans(  p1, previousPose_heading, walls_pc,      nullPose ,matchPose    )  ;
  alignTwoScans(  p1, previousPose_heading, raytrace_pc,      nullPose ,matchPose    )  ;
  // Add back in subtraction:
  matchPose = isam::Pose2d( matchPose.x() + previousPose.x(),
    matchPose.y() + previousPose.y(), matchPose.t() ); // heading not added back in
  
  //  Display Alignment
  int match_coll =1;
  int match_id = 123456566;
  string match_name = "Matched Pose [G]";
  sendPose(matchPose,match_coll,match_id,match_name);
  int match_scan_coll = 2;
  string match_scan_name = "Matched Scan [G]";
  sendScan(p1,match_coll,match_id,match_scan_coll, match_scan_name );
  
//  previousPose = matchPose;
  return matchPose;
}


// Manually Align the galign scan using the scanmatchers best estimate:
void AlignUtils::galignScanManual(PointCloudPtr & scan,isam::Pose2d &galignment){
  cout << "Steer manual alignment:\n";
  cout << "o/p - rotate\n";
  cout << "z/c - x direction\n";
  cout << "s/x - y direction\n";
  cout << "k - keep/accept and exit\n";
  char c;
  double scale[3] = {0.01,0.01,0.0025}; 
  // was {0.01,0.01,0.005}; 
  // changed to {0.01,0.01,0.0025}; in aug 2012 
  // 1cm and 0.25 degrees
  std::system ("/bin/stty raw"); // use system call to make terminal send all keystrokes directly to stdin
  while((c=getchar())!= 'k') { // type a period to break out of the loop, since CTRL-D won't work raw
    putchar(c);
    if (c == 'z'){ // -x
      galignment = isam::Pose2d(galignment.x() - scale[0], galignment.y(), galignment.t());
    }else if(c == 'c'){ // x
      galignment = isam::Pose2d(galignment.x() + scale[0], galignment.y(), galignment.t());
    }else if(c == 's'){ // y
      galignment = isam::Pose2d(galignment.x(), galignment.y() + scale[1], galignment.t());
    }else if(c == 'x'){ //-y
      galignment = isam::Pose2d(galignment.x(), galignment.y() - scale[1], galignment.t());
    }else if(c == 'o'){ // -t
      galignment = isam::Pose2d(galignment.x(), galignment.y(), galignment.t() - scale[2] );
    }else if(c == 'p'){ // t
      galignment = isam::Pose2d(galignment.x(), galignment.y(), galignment.t() + scale[2] );
    }

    int pose_collg =1250;
    int pose_idg = 132345678;
    string pose_nameg = "Galign Pose [G]";
    sendPose(galignment,pose_collg,pose_idg,pose_nameg);
    int scan_collg = 1251;
    string scan_nameg = "Galign Scan [G]";
    sendScan(scan,pose_collg,pose_idg,scan_collg, scan_nameg );  

  }
  std::system ("/bin/stty cooked"); // use system call to set terminal behaviour to more normal behaviour
  
  char temp;
  cout << "\n\nManual Alignment:\n" 
	<< "(" << galignment.x() << ", " << galignment.y() << ", " << galignment.t() << ")\n";
  cin >>temp;      
}




//
void AlignUtils::tweakScanManual(vector < SmoothBlock > &blocks,
    int which_block,
    double rad0, double radstep,
    vector <isam::Pose2d> &poses){

  int do_smooth=1;
  string smooth_string;
  if (do_smooth){
    smooth_string = " [Smoothed]";
  }else{
    smooth_string = " [Not Smth]";
  }

  // 1. Extract Global Alignments:
  vector <isam::Pose2d> global_poses;
  global_poses.push_back( blocks[0].first_pose );
  vector < vector < float> > global_scans;
  vector < float> empty_scan;
  global_scans.push_back(empty_scan); // 1st scan is avalaible
  for (size_t i=0; i<blocks.size(); i++){
    global_poses.push_back( blocks[i].last_pose );
    global_scans.push_back( blocks[i].last_scan );
  }
  vector< PointCloudPtr > global_scan_clouds;
  for (size_t i=0; i<global_poses.size(); i++){
    double validBeamAngles[] = {rad0, - rad0}; // not sure about this...
    PointCloudPtr p1 = convertToPointCloud(global_scans[i], 29.7, rad0, radstep, validBeamAngles);
    global_scan_clouds.push_back(p1);
  }

  // 2. Put the global alignments to the viewer:
  int global_pose_coll = 360;
  int global_pose_id = 10000000;
  string global_name = "Original Global Poses";
  sendPoseVector(global_poses, global_pose_coll, global_pose_id, global_name, VS_OBJ_COLLECTION_T_AXIS3D );
  int global_scan_coll = 361;
  string global_scan_name = "Original Global Scans";
  sendScanVector(global_scan_clouds, global_pose_coll, global_pose_id, global_scan_coll, global_scan_name, true);


  // 3. Generate the scan_clouds by resmoothing: [awkward]
  //vector <isam::Pose2d> poses;
  vector < int64_t > pose_utimes;
  vector < vector <float> > scans;
  smooth_log(blocks);
  convert_smoothed_log_to_output(poses,pose_utimes, scans, blocks, do_smooth);
  vector< PointCloudPtr > scan_clouds;
  for (size_t i=0; i<scans.size(); i++){
    double validBeamAngles[] = {rad0, - rad0}; // not sure about this...
    PointCloudPtr p1 = convertToPointCloud(scans[i], 29.7, rad0, radstep, validBeamAngles);
    scan_clouds.push_back(p1);
  }


  // Scans sent once back in main program
  int pose_coll = 22220+ do_smooth;
  int pose_id = 0;
  string pose_name = "Adjusted Poses" + smooth_string;
  //int scans_coll = 22220+1+do_smooth;
  //string scans_name = "Adjusted Scans"+ smooth_string;
  //sendScanVector(scan_clouds, pose_coll, pose_id, scans_coll, scans_name,true );


  cout << "Steer manual alignment:\n";
  cout << "o/p rotate | z/c x-dir | s/x y-dir | k keep & exit\n";
  char c;
  double scale[3] = {0.01,0.01,0.0025};
  // was {0.01,0.01,0.005};
  // changed to {0.01,0.01,0.0025}; in aug 2012
  // 1cm and 0.25 degrees
  std::system ("/bin/stty raw"); // use system call to make terminal send all keystrokes directly to stdin
  while((c=getchar())!= 'k') { // type a period to break out of the loop, since CTRL-D won't work raw
    putchar(c);

    // 1. Take a global pose and adjust it:
    isam::Pose2d galignment = global_poses[which_block];
    PointCloudPtr scan = global_scan_clouds[which_block];

    if (c == 'z'){ // -x
      galignment = isam::Pose2d(galignment.x() - scale[0], galignment.y(), galignment.t());
    }else if(c == 'c'){ // x
      galignment = isam::Pose2d(galignment.x() + scale[0], galignment.y(), galignment.t());
    }else if(c == 's'){ // y
      galignment = isam::Pose2d(galignment.x(), galignment.y() + scale[1], galignment.t());
    }else if(c == 'x'){ //-y
      galignment = isam::Pose2d(galignment.x(), galignment.y() - scale[1], galignment.t());
    }else if(c == 'o'){ // -t
      galignment = isam::Pose2d(galignment.x(), galignment.y(), galignment.t() - scale[2] );
    }else if(c == 'p'){ // t
      galignment = isam::Pose2d(galignment.x(), galignment.y(), galignment.t() + scale[2] );
    }

      struct timespec t;
      t.tv_sec = 1; // mfallon: was 0
      t.tv_nsec = (time_t)(50000000.5 * 1E9);  // was 0.5
      nanosleep(&t, NULL);

    // 2. Apply change to the data structures:
    // Also smooth only the relevent blocks:
    global_poses[which_block] = galignment;
    if (which_block>0){
      int w1 = which_block-1;
      blocks[w1].last_pose = galignment;
      std::vector<isam::Pose2d> smoothed_poses;
      smooth_poses(blocks[w1].first_pose, blocks[w1].last_pose, blocks[w1].incremental_poses, &smoothed_poses);
      blocks[w1].smoothed_poses.clear();
      for (size_t j = 0; j < smoothed_poses.size(); j++){
        Pose2d pose = smoothed_poses[j];
        blocks[w1].smoothed_poses.push_back(pose);
      }
      blocks[w1].smoothed_poses.push_back(blocks[w1].last_pose);
    }

    if (which_block< blocks.size()){
      int w2 = which_block;
      blocks[w2].first_pose = galignment;
      std::vector<isam::Pose2d> smoothed_poses;
      smooth_poses(blocks[w2].first_pose, blocks[w2].last_pose, blocks[w2].incremental_poses, &smoothed_poses);
      blocks[w2].smoothed_poses.clear();
      for (size_t j = 0; j < smoothed_poses.size(); j++){
        Pose2d pose = smoothed_poses[j];
        blocks[w2].smoothed_poses.push_back(pose);
      }
      blocks[w2].smoothed_poses.push_back(blocks[w2].last_pose);
    }

    // 3. Re-smooth:
    // TODO: only resmooth the required blocks
    poses.clear();
    pose_utimes.clear();
    vector < vector <float> > scans_unused; // (as the point cloud is reused below
    //smooth_log(blocks);
    convert_smoothed_log_to_output(poses,pose_utimes, scans, blocks, do_smooth);

    // 4a. Visualise that global pose and its scan:
    int pose_collg =200;
    int pose_idg = 132345678;
    string pose_nameg = "Galign Pose [G]";
    sendPose(galignment,pose_collg,pose_idg,pose_nameg);
    int scan_collg = 201;
    string scan_nameg = "Galign Scan [G]";
    sendScan(scan,pose_collg,pose_idg,scan_collg, scan_nameg );

    // 4b. Visualize the global poses and scans:
    int global_pose_coll = 260;
    int global_pose_id = 10000000;
    string global_name = "Adjusted Global Poses";
    sendPoseVector(global_poses, global_pose_coll, global_pose_id, global_name, VS_OBJ_COLLECTION_T_AXIS3D);
    int global_scan_coll = 261;
    string global_scan_name = "Adjusted Global Scans";
    sendScanVector(global_scan_clouds, global_pose_coll, global_pose_id, global_scan_coll, global_scan_name,true );

    string global_ids_name = "Adjusted Global Poses [Ids]";
    int global_text_coll =262;
    sendPoseIdVector(global_scan_clouds.size(), global_pose_coll, global_pose_id, global_text_coll, global_ids_name);

    // 4c. Output The full solution to the viewer:
    sendPoseVector(poses, pose_coll, pose_id, pose_name, VS_OBJ_COLLECTION_T_AXIS3D );
  }
  std::system ("/bin/stty cooked"); // use system call to set terminal behaviour to more normal behaviour
}




// This function is a clone of ScanMatcherUtils.sm_projectRangesAndDecimate but
// providing x,y pairs 
// I'm not actually using it currently but it improves alignment by using longer range points
// more and shorter ranges less.
int AlignUtils::projectUntiltedRangesAndDecimate(int beamskip, float spatialDecimationThresh,
        float * x,float * y, int numPoints, double thetaStart, double thetaStep,
        smPoint * points, double maxRange = 1e10, double validRangeStart =
                -1000, double validRangeEnd = 1000)
{
  int lastAdd = -1000;
  // double aveRange;
  double stdDevRange;    
  
  // start of replacement for ScanMatcherUtils.sm_projectRangesToPoints
  int count = 0;
  double aveRange = 0;
  double aveRangeSq = 0;

  double theta = thetaStart;
  double this_range;
  for (int i = 0; i < numPoints; i++) {
      this_range = x[i]/cos(theta);
      if (this_range > .1 && this_range < maxRange && theta > validRangeStart
	      && theta < validRangeEnd) { //hokuyo driver seems to report maxRanges as .001 :-/
	  //project to body centered coordinates
	  points[count].x = x[i];
	  points[count].y = y[i];
	  count++;
	  aveRange += this_range;
	  aveRangeSq += sm_sq(this_range);
      }
      theta += thetaStep;
  }
  aveRange /= (double) count;
  aveRangeSq /= (double) count;

  // below is equivlent to the output of sm_projectRangesToPoints
  //aveRange = aveRange;
  stdDevRange = sqrt(aveRangeSq - sm_sq(aveRange));
  int numValidPoints = count;

  smPoint origin =       { 0, 0 };
  smPoint lastAddPoint =      { 0, 0 };
  int numDecPoints = 0;
  for (int i = 0; i < numValidPoints; i++) {
      //add every beamSkip beam, unless points are more than spatialDecimationThresh, or more than 1.8 stdDevs more than ave range
      if ((i - lastAdd) > beamskip || sm_dist(&points[i], &lastAddPoint)
	      > spatialDecimationThresh || sm_dist(&points[i], &origin)
	      > (aveRange + 1.8 * stdDevRange)) {
	  lastAdd = i;
	  lastAddPoint = points[i];
	  points[numDecPoints] = points[i]; // ok since i >= numDecPoints
	  numDecPoints++;
      }
  }

  return numDecPoints;
}


//PointCloudPtr AlignUtils::convertToPointCloud(float * lidar_ranges){
PointCloudPtr AlignUtils::convertToPointCloud(vector <float> ranges_vector,
  double maxRange, double rad0, double radstep, double validBeamAngles[]){
  PointCloudPtr pc = boost::make_shared<PointCloud>();
  
  int nranges = ranges_vector.size();

  float * ranges_array = (float *) calloc(nranges, sizeof(smPoint));
  for (int i=0; i<nranges ; ++i) {
    ranges_array[i] = ranges_vector[i];
  }  
    
  //int nranges= 1081;
  //double maxRange = 29.7;
  // double validBeamAngles[] ={-2.1,2.1};// not sure about this...

  double spatialDecimationThresh = 0.2;
  int beam_skip=3;
  smPoint * points = (smPoint *) calloc(nranges, sizeof(smPoint));
  int numValidPoints = sm_projectRangesAndDecimate(beam_skip,
      spatialDecimationThresh, ranges_array, nranges, rad0,
      radstep, points, maxRange, validBeamAngles[0],
      validBeamAngles[1]);
  for (int i=0; i<numValidPoints; ++i) 
    pc->addPoint(points[i].x, points[i].y, 0.0);
  
  free(points);
  free(ranges_array);
  
  return pc;
}



int AlignUtils::alignTwoScans( PointCloudPtr & currentPointCloud,
      isam::Pose2d currentPose,
      PointCloudPtr & proposalPointCloud,
      isam::Pose2d proposalPose,  isam::Pose2d & matchPose){
  
  bool verboseClosure =false;
    
  int no_matches =0; //output indicator: 0 if no match, 1 if one was found

  // 1. Set hardcoded scan matcher params
  double metersPerPixel = .02;//.02 //translational resolution for the brute force search
  double thetaResolution = .01;//.01 //angular step size for the brute force search
  int useGradientAscentPolish = 1; //use gradient descent to improve estimate after brute force search
  int useMultires = 3; // low resolution will have resolution metersPerPixel * 2^useMultiRes


  int maxNumScans = 30; //keep around this many scans in the history
  double addScanHitThresh = .80; //add a new scan to the map when the number of "hits" drops below this
  
  int useThreads = 0;
  bool stationaryMotionModel = true;// was false
  double motionModelPriorWeight = 0.0; //was 0.2
  scanmatch::ScanTransform startPose;
  memset(&startPose, 0, sizeof(startPose));

  scanmatch::ScanMatcher* sm_loop = NULL;

  scanmatch::sm_incremental_matching_modes_t matchingMode;
  matchingMode = scanmatch::SM_GRID_COORD; // SM_GRID_ONLY;

  // Scanmatcher settings were updated...
  // Set search range for incremental scan matching
  //SHOULD be set greater than the initialSearchRange
  double initialSearchRangeXY = .15; //nominal range that will be searched over
  double initialSearchRangeTheta = .1;
  double maxSearchRangeXY = .3; //if a good match isn't found I'll expand and try again up to this size...
  double maxSearchRangeTheta = .2; //if a good match isn't found I'll expand and try again up to this size...
  
  // Start of new function	
  if (sm_loop) delete sm_loop;
  sm_loop = new scanmatch::ScanMatcher(metersPerPixel, thetaResolution, useMultires, useThreads, verboseClosure);
  //sm_loop->initSuccessiveMatchingParams(maxNumScans, initialSearchRangeXY, maxSearchRangeXY,
  //initialSearchRangeTheta, maxSearchRangeTheta, matchingMode, addScanHitThresh,
  //stationaryMotionModel, motionModelPriorWeight, &startPose);
  memset(&sm_loop->currentPose, 0, sizeof(sm_loop->currentPose));
  
  // 2. Put the proposal into SM, if its too small quit:
  PointCloudPtr proposalScan = proposalPointCloud;//.data;
//  MapNodePtr proposalNode = proposalPointCloud.node;

  int numValidPoints = proposalScan->size();
  if (numValidPoints < 30) {
    std::cerr << "LOOP: WARNING! NOT ENOUGH VALID POINTS! numValid=" << numValidPoints << std::endl;
    no_matches =0;
    return no_matches;
  }

  scanmatch::smPoint* points0 = new scanmatch::smPoint[proposalScan->size()];
  for (size_t l=0; l<proposalScan->size(); l++) {
    points0[l].x = proposalScan->points()[l].x();
    points0[l].y = proposalScan->points()[l].y();
  }
  
  
  ScanTransform trans0;
  memset(&trans0, 0, sizeof(trans0));
  
  cout << trans0.x << " " << trans0.y << " " << trans0.theta << " trans0\n";
  sm_loop->addScan(points0,numValidPoints,&trans0, scanmatch::SM_HOKUYO_UTM, 
		   scanmatch::sm_get_utime(), true);
  
/*
  scanmatch::ScanTransform r0 = sm_loop->matchSuccessive(points0, numValidPoints,
  scanmatch::SM_SICK_LMS, scanmatch::sm_get_utime(), false, NULL);
  */
  
  
  
  
  
  
//  scanmatch::SM_SICK_LMS, 123456, false, NULL);

  // 3. Put in current scan and get SM alignment and score
  scanmatch::ScanTransform p;
  memset(&p,0,sizeof(p));
  isam::Pose2d pdelta = currentPose.ominus(proposalPose);
  p.x = pdelta.x(); p.y = pdelta.y();
  p.theta = pdelta.t();
  p.score = 0.0; // set score on the prior
  // TODO: see if it should be based on our actual prior from the estimated map
  //std::cout << "proposalPose: " << proposalPose << std::endl;
  //std::cout << "currentPose: " << currentPose << std::endl;
  
  if (verboseClosure){
  std::cout << "Proposal DELTA: " << pdelta << std::endl;
  }
  
  // @todo use covariance to control search range
  initialSearchRangeXY = 1.0;
  initialSearchRangeTheta = 0.5;
  maxSearchRangeXY = 3.0;
  maxSearchRangeTheta = 1.5;

  scanmatch::sm_incremental_matching_modes_t matchingMode2;
  matchingMode2 = scanmatch::SM_GRID_COORD; // not sure why this is a different mode...

//  sm_loop->initSuccessiveMatchingParams(maxNumScans, initialSearchRangeXY, maxSearchRangeXY,
//  initialSearchRangeTheta, maxSearchRangeTheta, matchingMode2, addScanHitThresh,
//  stationaryMotionModel, motionModelPriorWeight, &startPose);

  scanmatch::smPoint* points = new scanmatch::smPoint[currentPointCloud->size()];
  for (size_t l=0; l<currentPointCloud->size(); l++) {
    points[l].x = currentPointCloud->points()[l].x();
    points[l].y = currentPointCloud->points()[l].y();
  }
  
  cout << currentPose.x() << " " << currentPose.y() << " " << currentPose.t() << " current pose in\n";
  cout << proposalPose.x() << " " << proposalPose.y() << " " << proposalPose.t() << " proposalPose pose in\n";
  
  scanmatch::ScanTransform tran1_guess;
  memset(&tran1_guess,0,sizeof(tran1_guess));
  tran1_guess.x = currentPose.x();
  tran1_guess.y =currentPose.y();
  tran1_guess.theta =  currentPose.t();
  
  scanmatch::ScanTransform tran1_intermediate = sm_loop->gridMatch(points, 
	  currentPointCloud->size(), &tran1_guess, 
          0.10, 0.10, 0.1, NULL, NULL, NULL); // tight
          //1.0, 1.0, 1.0, NULL, NULL, NULL); // very loose
  // search region in x y theta
  cout << tran1_guess.x << " " << tran1_guess.y << " " << tran1_guess.theta << " trans1_guess_out\n";
  cout << tran1_intermediate.x << " " << tran1_intermediate.y << " " << tran1_intermediate.theta << " tran1_intermediate\n";
  
  ScanTransform r = sm_loop->coordAscentMatch(points,
	    currentPointCloud->size(), &tran1_intermediate);  
  cout << r.x << " " << r.y << " " << r.theta << " r out\n";
  

  /*
  scanmatch::ScanTransform r = sm_loop->matchSuccessive(points,
  currentPointCloud->size(), scanmatch::SM_SICK_LMS,
  scanmatch::sm_get_utime(), true, &p);
  */

  if (verboseClosure){
    std::cout << "LOOP: Registration covariance " <<
    r.sigma[0] << " " << r.sigma[4] << " " << r.sigma[8] <<
    " " << (r.sigma[0]+r.sigma[4]+r.sigma[8])	<< std::endl;
  }

  double covTrace = r.sigma[0]+r.sigma[4]+r.sigma[8];

  // 4. Test the proposed aligment in a number of ways
  bool manual = false;
  if (manual){
    bool accept;
    if (SHOW_GUI) sm_loop->drawGUI(points, currentPointCloud->size(), r, NULL, "Loop Closure");
    if (verboseClosure) std::cout << "LOOP: new loopclosure (a-accept r-reject)" << std::endl;
    char c = getchar();
    if (c=='a') {
      getchar();
      if (verboseClosure) std::cout << "LOOP: loopclosure accepted" << std::endl;
      accept = true;
    } else {
      getchar();
      if (verboseClosure) std::cout << "LOOP: loopclosure rejected" << std::endl;
      accept = false;
    }

    if (accept) {
      if (verboseClosure) std::cout << "LOOP: Loop closure accepted: " << r.x << " " << r.y << " " << r.theta << " " << r.score << " " << r.hits << " " << (double)r.hits/currentPointCloud->size() << std::endl;
      //if (SHOW_GUI) sm_loop->drawGUI(points, currentPointCloud.data->size(), r, NULL, "Loop Closure Accepted");
      //m.setAccepted(true);
    } else {
      if (verboseClosure) std::cout << "LOOP: Loop closure rejected: " << r.x << " " << r.y << " " << r.theta << " " << r.score << " " << r.hits << " " << (double)r.hits/currentPointCloud->size() << std::endl;
      //if (SHOW_GUI) sm_loop->drawGUI(points, currentPointCloud.data->size(), r, NULL, "Loop Closure Rejected");
      //m.setAccepted(false);
      //m.setNoAdd(true);  // Never add a failed loop constraint
    }
  }  else  {
    if (   (double)r.hits/currentPointCloud->size() > 0.7
	&& covTrace < 0.01)
    //                if (   (double)r.hits/currentPointCloud.data->size() > 0.5
    //                        && covTrace < 0.001)
    //						&& fabs(m.x-p.x) < 0.2
    //						&& fabs(m.y-p.y) < 0.2
    //						&& fabs(m.t-p.theta) < 0.05 )
    {
      if (verboseClosure) std::cout << "LOOP: Loop closure accepted: " << r.x << " " << r.y << " " << r.theta << " " << r.score << " " << r.hits << " " << (double)r.hits/currentPointCloud->size() << std::endl;
      if (SHOW_GUI) sm_loop->drawGUI(points, currentPointCloud->size(), r, NULL, "galignScan");
      //m.setAccepted(true);
    } else {
      if (verboseClosure) std::cout << "LOOP: Loop closure rejected: " << r.x << " " << r.y << " " << r.theta << " " << r.score << " " << r.hits << " " << (double)r.hits/currentPointCloud->size() << std::endl;
      if (SHOW_GUI) sm_loop->drawGUI(points, currentPointCloud->size(), r, NULL, "galignScan");
      //m.setAccepted(false);
      //m.setNoAdd(true);  // Never add a failed loop constraint
    }
  }
  
  delete [] points;

  delete sm_loop;
  sm_loop = NULL;

  isam::Pose2d deltaPose(r.x,r.y,r.theta);
  matchPose = deltaPose;
  if (verboseClosure){
    std::cout << "LOOP: suggested global pose: " << matchPose.x() << " " << matchPose.y() << " " << matchPose.t() << "\n"; 
  }
  return no_matches;
}


#endif  
