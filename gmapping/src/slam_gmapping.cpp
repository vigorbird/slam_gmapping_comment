/*
 * slam_gmapping
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 * 
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Brian Gerkey */
/* Modified by: Charles DuHadway */


/**

@mainpage slam_gmapping

@htmlinclude manifest.html

@b slam_gmapping is a wrapper around the GMapping SLAM library. It reads laser
scans and odometry and computes a map. This map can be
written to a file using e.g.

  "rosrun map_server map_saver static_map:=dynamic_map"

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "scan"/<a href="../../sensor_msgs/html/classstd__msgs_1_1LaserScan.html">sensor_msgs/LaserScan</a> : data from a laser range scanner 
- @b "/tf": odometry from the robot


Publishes to (name/type):
- @b "/tf"/tf/tfMessage: position relative to the map


@section services
 - @b "~dynamic_map" : returns the map


@section parameters ROS parameters

Reads the following parameters from the parameter server

Parameters used by our GMapping wrapper:

- @b "~throttle_scans": @b [int] throw away every nth laser scan
- @b "~base_frame": @b [string] the tf frame_id to use for the robot base pose
- @b "~map_frame": @b [string] the tf frame_id where the robot pose on the map is published
- @b "~odom_frame": @b [string] the tf frame_id from which odometry is read
- @b "~map_update_interval": @b [double] time in seconds between two recalculations of the map


Parameters used by GMapping itself:

Laser Parameters:
- @b "~/maxRange" @b [double] maximum range of the laser scans. Rays beyond this range get discarded completely. (default: maximum laser range minus 1 cm, as received in the the first LaserScan message)
- @b "~/maxUrange" @b [double] maximum range of the laser scanner that is used for map building (default: same as maxRange)
- @b "~/sigma" @b [double] standard deviation for the scan matching process (cell)
- @b "~/kernelSize" @b [double] search window for the scan matching process
- @b "~/lstep" @b [double] initial search step for scan matching (linear)
- @b "~/astep" @b [double] initial search step for scan matching (angular)
- @b "~/iterations" @b [double] number of refinement steps in the scan matching. The final "precision" for the match is lstep*2^(-iterations) or astep*2^(-iterations), respectively.
- @b "~/lsigma" @b [double] standard deviation for the scan matching process (single laser beam)
- @b "~/ogain" @b [double] gain for smoothing the likelihood
- @b "~/lskip" @b [int] take only every (n+1)th laser ray for computing a match (0 = take all rays)
- @b "~/minimumScore" @b [double] minimum score for considering the outcome of the scanmatching good. Can avoid 'jumping' pose estimates in large open spaces when using laser scanners with limited range (e.g. 5m). (0 = default. Scores go up to 600+, try 50 for example when experiencing 'jumping' estimate issues)

Motion Model Parameters (all standard deviations of a gaussian noise model)
- @b "~/srr" @b [double] linear noise component (x and y)
- @b "~/stt" @b [double] angular noise component (theta)
- @b "~/srt" @b [double] linear -> angular noise component
- @b "~/str" @b [double] angular -> linear noise component

Others:
- @b "~/linearUpdate" @b [double] the robot only processes new measurements if the robot has moved at least this many meters
- @b "~/angularUpdate" @b [double] the robot only processes new measurements if the robot has turned at least this many rads

- @b "~/resampleThreshold" @b [double] threshold at which the particles get resampled. Higher means more frequent resampling.
- @b "~/particles" @b [int] (fixed) number of particles. Each particle represents a possible trajectory that the robot has traveled

Likelihood sampling (used in scan matching)
- @b "~/llsamplerange" @b [double] linear range
- @b "~/lasamplerange" @b [double] linear step size
- @b "~/llsamplestep" @b [double] linear range
- @b "~/lasamplestep" @b [double] angular step size

Initial map dimensions and resolution:
- @b "~/xmin" @b [double] minimum x position in the map [m]
- @b "~/ymin" @b [double] minimum y position in the map [m]
- @b "~/xmax" @b [double] maximum x position in the map [m]
- @b "~/ymax" @b [double] maximum y position in the map [m]
- @b "~/delta" @b [double] size of one pixel [m]

*/



#include "slam_gmapping.h"

#include <iostream>

#include <time.h>

#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/MapMetaData.h"

#include "gmapping/sensor/sensor_range/rangesensor.h"
#include "gmapping/sensor/sensor_odometry/odometrysensor.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

SlamGMapping::SlamGMapping():
  map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ))),
  laser_count_(0), private_nh_("~"), scan_filter_sub_(NULL), scan_filter_(NULL), transform_thread_(NULL)
{
   //time(NULL)返回从1970年元旦午夜0点到现在的秒数
   //只在initMapper中被调用过
  seed_ = time(NULL);
  init();
}

SlamGMapping::SlamGMapping(long unsigned int seed, long unsigned int max_duration_buffer):
  map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ))),
  laser_count_(0), private_nh_("~"), scan_filter_sub_(NULL), scan_filter_(NULL), transform_thread_(NULL),
  seed_(seed), tf_(ros::Duration(max_duration_buffer))
{
  init();
}
 

void SlamGMapping::init()
{
  // log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  // The library is pretty chatty
  //gsp_ = new GMapping::GridSlamProcessor(std::cerr);
  gsp_ = new GMapping::GridSlamProcessor(); //调用其构造函数初始化三个参数
  ROS_ASSERT(gsp_);//表达式结果如果为false程序将停止执行

  tfB_ = new tf::TransformBroadcaster();//这个主要用来发布坐标变换
  ROS_ASSERT(tfB_);

  gsp_laser_ = NULL;
  gsp_odom_ = NULL;

  got_first_scan_ = false;
  got_map_ = false;
  

  
  // Parameters used by our GMapping wrapper
  if(!private_nh_.getParam("throttle_scans", throttle_scans_))
    throttle_scans_ = 1;
  if(!private_nh_.getParam("base_frame", base_frame_))
    base_frame_ = "base_link";
  if(!private_nh_.getParam("map_frame", map_frame_))
    map_frame_ = "map";
  if(!private_nh_.getParam("odom_frame", odom_frame_))
    odom_frame_ = "odom";

  private_nh_.param("transform_publish_period", transform_publish_period_, 0.05);

  double tmp;
  if(!private_nh_.getParam("map_update_interval", tmp))
    tmp = 5.0;
  map_update_interval_.fromSec(tmp);//将设置的参数(单位为秒)转化为ros持续时间
  
  // Parameters used by GMapping itself
  maxUrange_ = 0.0;  maxRange_ = 0.0; // preliminary default, will be set in initMapper()
  if(!private_nh_.getParam("minimumScore", minimum_score_))
    minimum_score_ = 0;
  if(!private_nh_.getParam("sigma", sigma_))
    sigma_ = 0.05;
  if(!private_nh_.getParam("kernelSize", kernelSize_))
    kernelSize_ = 1;
  if(!private_nh_.getParam("lstep", lstep_))
    lstep_ = 0.05;
  if(!private_nh_.getParam("astep", astep_))
    astep_ = 0.05;
  if(!private_nh_.getParam("iterations", iterations_))
    iterations_ = 5;
  if(!private_nh_.getParam("lsigma", lsigma_))
    lsigma_ = 0.075;
  if(!private_nh_.getParam("ogain", ogain_))
    ogain_ = 3.0;
  if(!private_nh_.getParam("lskip", lskip_))
    lskip_ = 0;
  if(!private_nh_.getParam("srr", srr_))
    srr_ = 0.1;
  if(!private_nh_.getParam("srt", srt_))
    srt_ = 0.2;
  if(!private_nh_.getParam("str", str_))
    str_ = 0.1;
  if(!private_nh_.getParam("stt", stt_))
    stt_ = 0.2;
  if(!private_nh_.getParam("linearUpdate", linearUpdate_))
    linearUpdate_ = 1.0;
  if(!private_nh_.getParam("angularUpdate", angularUpdate_))
    angularUpdate_ = 0.5;
  if(!private_nh_.getParam("temporalUpdate", temporalUpdate_))
    temporalUpdate_ = -1.0;
  if(!private_nh_.getParam("resampleThreshold", resampleThreshold_))
    resampleThreshold_ = 0.5;
  if(!private_nh_.getParam("particles", particles_))
    particles_ = 30;
  if(!private_nh_.getParam("xmin", xmin_))
    xmin_ = -100.0;
  if(!private_nh_.getParam("ymin", ymin_))
    ymin_ = -100.0;
  if(!private_nh_.getParam("xmax", xmax_))
    xmax_ = 100.0;
  if(!private_nh_.getParam("ymax", ymax_))
    ymax_ = 100.0;
  if(!private_nh_.getParam("delta", delta_))
    delta_ = 0.05;
  if(!private_nh_.getParam("occ_thresh", occ_thresh_))
    occ_thresh_ = 0.25;
  if(!private_nh_.getParam("llsamplerange", llsamplerange_))
    llsamplerange_ = 0.01;
  if(!private_nh_.getParam("llsamplestep", llsamplestep_))
    llsamplestep_ = 0.01;
  if(!private_nh_.getParam("lasamplerange", lasamplerange_))
    lasamplerange_ = 0.005;
  if(!private_nh_.getParam("lasamplestep", lasamplestep_))
    lasamplestep_ = 0.005;
    
  if(!private_nh_.getParam("tf_delay", tf_delay_))
    tf_delay_ = transform_publish_period_;
  

}


void SlamGMapping::startLiveSlam()
{
  //下面为发布的topic
  entropy_publisher_ = private_nh_.advertise<std_msgs::Float64>("entropy", 1, true);
  sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  ss_ = node_.advertiseService("dynamic_map", &SlamGMapping::mapCallback, this);
  
  //订阅scan topic，并且接收到的数据要转换到的目标坐标系是odom_frame_
  //一旦tf树中有odom_frame_的坐标系信息就调用函数laserCallback
  scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "scan", 5);
  scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5);
  scan_filter_->registerCallback(boost::bind(&SlamGMapping::laserCallback, this, _1));//一旦里程计坐标系发布到tf中则调用回调函数

 //开启一个线程
 //默认的transform_publish_period_=0.05，
  transform_thread_ = new boost::thread(boost::bind(&SlamGMapping::publishLoop, this, transform_publish_period_));
}

void SlamGMapping::startReplay(const std::string & bag_fname, std::string scan_topic)
{
  double transform_publish_period;
  ros::NodeHandle private_nh_("~");
  entropy_publisher_ = private_nh_.advertise<std_msgs::Float64>("entropy", 1, true);
  sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  ss_ = node_.advertiseService("dynamic_map", &SlamGMapping::mapCallback, this);
  
  rosbag::Bag bag;
  bag.open(bag_fname, rosbag::bagmode::Read);
  
  std::vector<std::string> topics;
  topics.push_back(std::string("/tf"));
  topics.push_back(scan_topic);
  rosbag::View viewall(bag, rosbag::TopicQuery(topics));

  // Store up to 5 messages and there error message (if they cannot be processed right away)
  std::queue<std::pair<sensor_msgs::LaserScan::ConstPtr, std::string> > s_queue;
  foreach(rosbag::MessageInstance const m, viewall)
  {
    tf::tfMessage::ConstPtr cur_tf = m.instantiate<tf::tfMessage>();
    if (cur_tf != NULL) {
      for (size_t i = 0; i < cur_tf->transforms.size(); ++i)
      {
        geometry_msgs::TransformStamped transformStamped;
        tf::StampedTransform stampedTf;
        transformStamped = cur_tf->transforms[i];
        tf::transformStampedMsgToTF(transformStamped, stampedTf);
        tf_.setTransform(stampedTf);
      }
    }

    sensor_msgs::LaserScan::ConstPtr s = m.instantiate<sensor_msgs::LaserScan>();
    if (s != NULL) {
      if (!(ros::Time(s->header.stamp)).is_zero())
      {
        s_queue.push(std::make_pair(s, ""));
      }
      // Just like in live processing, only process the latest 5 scans
      if (s_queue.size() > 5) {
        ROS_WARN_STREAM("Dropping old scan: " << s_queue.front().second);
        s_queue.pop();
      }
      // ignoring un-timestamped tf data 
    }

    // Only process a scan if it has tf data
    while (!s_queue.empty())
    {
      try
      {
        tf::StampedTransform t;
        tf_.lookupTransform(s_queue.front().first->header.frame_id, odom_frame_, s_queue.front().first->header.stamp, t);
        this->laserCallback(s_queue.front().first);
        s_queue.pop();
      }
      // If tf does not have the data yet
      catch(tf2::TransformException& e)
      {
        // Store the error to display it if we cannot process the data after some time
        s_queue.front().second = std::string(e.what());
        break;
      }
    }
  }

  bag.close();
}

//transform_publish_period这个参数是全局参数，从launch文件中读取
void SlamGMapping::publishLoop(double transform_publish_period){
  if(transform_publish_period == 0)
    return;
  //设置ros系统循环的频率，单位为赫兹，系统默认的频率为20Hz
  ros::Rate r(1.0 / transform_publish_period);
  while(ros::ok()){
    publishTransform();
    r.sleep();
  }
}

SlamGMapping::~SlamGMapping()
{
  if(transform_thread_){
    transform_thread_->join();
    delete transform_thread_;
  }

  delete gsp_;
  if(gsp_laser_)
    delete gsp_laser_;
  if(gsp_odom_)
    delete gsp_odom_;
  if (scan_filter_)
    delete scan_filter_;
  if (scan_filter_sub_)
    delete scan_filter_sub_;
}

bool
SlamGMapping::getOdomPose(GMapping::OrientedPoint& gmap_pose, const ros::Time& t)
{
  // Get the pose of the centered laser at the right time
  centered_laser_pose_.stamp_ = t;
  // Get the laser's pose that is centered
  tf::Stamped<tf::Transform> odom_pose;
  try
  {
	//将坐标centered_laser_pose_(激光传感器坐标系)转换为odom_frame坐标系下的坐标，并将坐标保存在odom_pose中
	//前提是odom_frame已经由handfree平台发布到了tf树中
	//centered_laser_pose_只在initMapper函数中得到，并且是常数
	tf_.transformPose(odom_frame_, centered_laser_pose_, odom_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  double yaw = tf::getYaw(odom_pose.getRotation());

  gmap_pose = GMapping::OrientedPoint(odom_pose.getOrigin().x(),
                                      odom_pose.getOrigin().y(),
                                      yaw);
  return true;
}

bool
SlamGMapping::initMapper(const sensor_msgs::LaserScan& scan)
{
  laser_frame_ = scan.header.frame_id;
//************************************************************************
 //************************************************************************
 //************************************************************************
  // Get the laser's pose, relative to base.如何获取的高度信息????
  tf::Stamped<tf::Pose> ident;
  tf::Stamped<tf::Transform> laser_pose;
  ident.setIdentity();//set this transformation(变换) to identity(即单位矩阵)
  ident.frame_id_ = laser_frame_;
  ident.stamp_ = scan.header.stamp;//获取扫描数据的时间信息。stamp数据类型:time stamp
  try
  {
    //初始化时先用基座的位置信息来确定激光传感器的位置
    //基座的坐标系信息必须事先要发布到tf树中
    //将激光坐标系下的ident坐标转换成base坐标系的坐标，并将结果保存在laser_pose中
    //laser_pose这个参数只是用来判断激光传感器有没有装反。
    tf_.transformPose(base_frame_, ident, laser_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute laser pose, aborting initialization (%s)",
             e.what());
    return false;
  }

  // create a point 1m above the laser position and transform it into the laser-frame
  tf::Vector3 v;
  v.setValue(0, 0, 1 + laser_pose.getOrigin().z());//Return the origin vector translation
  tf::Stamped<tf::Vector3> up(v, scan.header.stamp,base_frame_);
  
  try
  {
    //将在base坐标系下的up坐标转换成laser_frame_坐标系下，并保存到up变量
    tf_.transformPoint(laser_frame_, up, up);
    ROS_DEBUG("Z-Axis in sensor frame: %.3f", up.z());
  }
  catch(tf::TransformException& e)
  {
    ROS_WARN("Unable to determine orientation of laser: %s",
             e.what());
    return false;
  }
  
  // gmapping doesnt take roll or pitch into account. So check for correct sensor alignment.
  if (fabs(fabs(up.z()) - 1) > 0.001)
  {
    ROS_WARN("Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f",
                 up.z());
    return false;
  }

  gsp_laser_beam_count_ = scan.ranges.size();
  //angle_min是第一个扫描的角度数据，angle_max是最后一个扫描的角度数据
  //如果我们使用的是rplidar，则angle_min=0，angle_max=359*pi/180
  double angle_center = (scan.angle_min + scan.angle_max)/2;//对应180度

  if (up.z() > 0)//什么意思??
  {
    do_reverse_range_ = scan.angle_min > scan.angle_max;
    //tf::Stamped<tf::Pose>结构中第一个参数设置yaw的角度，第二个参数是位置，第三个参数是时间，第四个参数是坐标系
    //centered_laser_pose_是激光传感器坐标系下的坐标
    centered_laser_pose_ = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(0,0,angle_center),
                                                               tf::Vector3(0,0,0)), ros::Time::now(), laser_frame_);
    ROS_INFO("Laser is mounted upwards.");
  }
  else
  {
    do_reverse_range_ = scan.angle_min < scan.angle_max;
    centered_laser_pose_ = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(M_PI,0,-angle_center),//M_PI是在C++中定义的3.1415926
                                                               tf::Vector3(0,0,0)), ros::Time::now(), laser_frame_);
    ROS_INFO("Laser is mounted upside down.");
  }

 //************************************************************************
 //************************************************************************
 //************************************************************************
  // Compute the angles of the laser from -x to x, basically symmetric and in increasing order
  laser_angles_.resize(scan.ranges.size());
  // Make sure angles are started so that they are centered
  // 初始化角度数据
  double theta = - std::fabs(scan.angle_min - scan.angle_max)/2;//如果我们使用的rplidar，则theta对应180度
  for(unsigned int i=0; i<scan.ranges.size(); ++i)
  {
    laser_angles_[i]=theta;
    theta += std::fabs(scan.angle_increment);//如果我们使用的是riplidar，angle_increment对应1度
  }

  ROS_DEBUG("Laser angles in laser-frame: min: %.3f max: %.3f inc: %.3f", scan.angle_min, scan.angle_max,
            scan.angle_increment);
  ROS_DEBUG("Laser angles in top-down centered laser-frame: min: %.3f max: %.3f inc: %.3f", laser_angles_.front(),
            laser_angles_.back(), std::fabs(scan.angle_increment));

  //************************************************************************
 //************************************************************************
 //************************************************************************
  GMapping::OrientedPoint gmap_pose(0, 0, 0);//pose relative to the sensor center,前两个参数是位置，最后一个参数是角度

  ros::NodeHandle private_nh_("~");//启动节点，命名空间就是节点名称
 // setting maxRange and maxUrange here so we can set a reasonable default 
  //maxRange_是传感器最大的测量距离，maxUrange_是最大的可用的有效测量距离
  //ros官网上建议set maxUrange < maximum range of the real sensor <= maxRange.
  if(!private_nh_.getParam("maxRange", maxRange_))
    maxRange_ = scan.range_max - 0.01;
  if(!private_nh_.getParam("maxUrange", maxUrange_))
    maxUrange_ = maxRange_;

  // The laser must be called "FLASER".这个是RangeSensor的名字
  // We pass in the absolute value of the computed angle increment, on the
  // assumption that GMapping requires a positive angle increment.  If the
  // actual increment is negative, we'll swap the order of ranges before
  // feeding each scan to GMapping.
  //初始化了传感器扫描的beam角度数据(m_beam)和此刻传感器的位置
  gsp_laser_ = new GMapping::RangeSensor("FLASER",
                                         gsp_laser_beam_count_,//gsp_laser_beam_count_用来初始化RangeSensor中m_beams元素个数
                                         fabs(scan.angle_increment),
                                         gmap_pose,//初始化RangeSensor::m_pose，激光传感器与小车底座的相对位置信息
                                         0.0,
                                         maxRange_);
  ROS_ASSERT(gsp_laser_);//声明这个提供的命令或者参数是真,否则程序会终止执行


  //GMapping::SensorMap 其实是std::map<std::string, Sensor*>，
  //这里的map不是表示地图而是一种特殊的数据结构:键值对
  GMapping::SensorMap smap;//这是一个局部变量!!!!!!!!
  //make_pair就是构造一个对象，第一个元素是gsp_laser_->getName()，第二个元素是gsp_laser_
  smap.insert(make_pair(gsp_laser_->getName(), gsp_laser_));
  gsp_->setSensorMap(smap);//主要作用是初始化变量m_matcher(ScanMatcher)中beam的角度、个数和激光传感器此刻的位置

  gsp_odom_ = new GMapping::OdometrySensor(odom_frame_);
  ROS_ASSERT(gsp_odom_);


  /// @todo Expose setting an initial pose
  GMapping::OrientedPoint initialPose;
  //将激光坐标系下的centered_laser_pose坐标，转换成里程计坐标系下的坐标，并保存在initialPose中  
  if(!getOdomPose(initialPose, scan.header.stamp))
  {
    ROS_WARN("Unable to determine inital pose of laser! Starting point will be set to zero.");
    initialPose = GMapping::OrientedPoint(0.0, 0.0, 0.0);
  }

//************************************************************************
 //************************************************************************
 //************************************************************************
//The sigma used by the greedy endpoint matching
//kernelSize_:The kernel in which to look for a correspondence
//lstep_:The optimization step in translation
//astep_:The optimization step in rotation
//iterations_:The number of iterations of the scanmatcher
//lsigma_:The sigma of a beam used for likelihood computation
//ogain_:Gain to be used while evaluating the likelihood, for smoothing the resampling effects
//lskip:Number of beams to skip in each scan.
//设置m_matcher参数，m_matcher是GridSlamProcessor类中定义的变量
  gsp_->setMatchingParameters(maxUrange_, maxRange_, sigma_,
                              kernelSize_, lstep_, astep_, iterations_,
                              lsigma_, ogain_, lskip_);
//srr_:Odometry error in translation as a function of translation
//srt_:Odometry error in translation as a function of rotation (rho/theta)
//str_:Odometry error in rotation as a function of translation (theta/rho)
//stt_:Odometry error in rotation as a function of rotation (theta/theta)
//设置m_motionModel参数，m_motionModel是GridSlamProcessor类中定义的变量
  gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
//linearUpdate_:Process a scan each time the robot translates this far
//angularUpdate_:Process a scan each time the robot rotates this far
//resampleThreshold_:The Neff based resampling threshold
//设置m_linearThresholdDistance,m_angularThresholdDistance,m_resampleThreshold???这些参数没有找到定义
  gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
//Process a scan if the last scan processed is older than the update time in seconds.
//A value less than zero will turn time based updates off.
//设置period_，period_是GridSlamProcessor类中定义的变量
  gsp_->setUpdatePeriod(temporalUpdate_);

  //!!!!!!!这个要尤其注意!!!!后面会用到!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //将GridSlamProcessor类中的m_generateMap=false
  gsp_->setgenerateMap(false);
  
  //particles_:Number of particles in the filter
  //xmin_,ymin_,xmax_, ymax_:Initial map size
  //delta_:Resolution of the map
  
  //!!!!这个函数要主要分析!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //搜索"void GridSlamProcessor::init"就能找到对应函数
  //initialPose是里程计坐标系下的激光传感器的坐标
  //默认值:xmin_=-100,xmax=100,particles_=30,delta_=0.05
  gsp_->GridSlamProcessor::init(particles_, xmin_, ymin_, xmax_, ymax_,
                                delta_, initialPose);
  
  //************************************************************************
 //************************************************************************
  //llsamplerange_:Translational sampling range for the likelihood
  //下面的几行代码都是使用宏定义的setllsamplerange等函数
  //都是对m_matcher中的参数进行设置
  gsp_->setllsamplerange(llsamplerange_);
  //llsamplestep_:Translational sampling step for the likelihood
  gsp_->setllsamplestep(llsamplestep_);
  /// @todo Check these calls; in the gmapping gui, they use
  /// llsamplestep and llsamplerange intead of lasamplestep and
  /// lasamplerange.  It was probably a typo, but who knows.
  //lasamplerange_:Angular sampling range for the likelihood
  gsp_->setlasamplerange(lasamplerange_);
  //lasamplestep_:Angular sampling step for the likelihood
  gsp_->setlasamplestep(lasamplestep_);
  //Minimum score for considering the outcome of the scan matching good. 
  //Can avoid jumping pose estimates in large open spaces when using laser scanners with limited range (e.g. 5m). 
  //Scores go up to 600+, try 50 for example when experiencing jumping estimate issues.
  gsp_->setminimumScore(minimum_score_);
  //************************************************************************
  //************************************************************************

  // Call the sampling function once to set the seed.
  //之前在构造SlamGMapping时定义的seed_ = time(NULL)，
  //本质上是调用了srand48函数,并将第一个参数作为方差(期望默认为0)，生成一个服从高斯分布的随机数
  //注意本来这个函数是有返回值的，返回值就是服从高斯的随机数，但是此处的目的主要是初始化随机序列
  GMapping::sampleGaussian(1,seed_);
  
  ROS_INFO("Initialization complete");

  return true;
}


bool
SlamGMapping::addScan(const sensor_msgs::LaserScan& scan, GMapping::OrientedPoint& gmap_pose)
{
  if(!getOdomPose(gmap_pose, scan.header.stamp))//根据里程计信息得到里程计在世界坐标系下的坐标gmap_pose
     return false;
  
  if(scan.ranges.size() != gsp_laser_beam_count_)
    return false;

  // GMapping wants an array of doubles...
  double* ranges_double = new double[scan.ranges.size()];
  // If the angle increment is negative, we have to invert the order of the readings.
  if (do_reverse_range_)//存储的扫描数据一定是从大到小
  {
	    ROS_DEBUG("Inverting scan");
	    int num_ranges = scan.ranges.size();
	    for(int i=0; i < num_ranges; i++)
	    {
	      // Must filter out short readings, because the mapper won't
	      if(scan.ranges[num_ranges - i - 1] < scan.range_min)
	        ranges_double[i] = (double)scan.range_max;
	      else
	        ranges_double[i] = (double)scan.ranges[num_ranges - i - 1];
	    }
  } 
  else//如果我们使用的是rplidar，则进入这个条件判断 
  {
	    for(unsigned int i=0; i < scan.ranges.size(); i++)
	    {
	      // Must filter out short readings, because the mapper won't
	      //如果获得的激光距离数据小于设定的允许最小值则将其值赋值为最大值
	      if(scan.ranges[i] < scan.range_min)
	        ranges_double[i] = (double)scan.range_max;
	      else
	        ranges_double[i] = (double)scan.ranges[i];
	    }
  }
 
  GMapping::RangeReading reading(scan.ranges.size(),
                                 ranges_double,
                                 gsp_laser_,//这个参数在initMapper中生成
                                 scan.header.stamp.toSec());

  // ...but it deep copies them in RangeReading constructor, so we don't
  // need to keep our array around.
  delete[] ranges_double;


  //gmap_pose这个参数是由函数刚开始时getOdomPose这个函数得到
  //里程计在世界坐标系下的坐标
  reading.setPose(gmap_pose);//这个位置信息是由上面的getOdomPose函数得到的

  /*
  ROS_DEBUG("scanpose (%.3f): %.3f %.3f %.3f\n",
            scan.header.stamp.toSec(),
            gmap_pose.x,
            gmap_pose.y,
            gmap_pose.theta);
            */
  ROS_DEBUG("processing scan");

  return gsp_->processScan(reading);
}


//一旦里程计的位置发布到tf树后自动调用这个函数，其输入的参数是激光扫描的数据
void
SlamGMapping::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  laser_count_++;
  if ((laser_count_ % throttle_scans_) != 0)//throttle_scans_默认值=1
    return;

  static ros::Time last_map_update(0,0);//第一个单位是秒 第二个单位是纳秒

  // We can't initialize the mapper until we've got the first scan
  if(!got_first_scan_)
  {
    if(!initMapper(*scan))
      return;
    got_first_scan_ = true;
  }

  GMapping::OrientedPoint odom_pose;
  
  //addScan函数的主要作用是读取scan的数据并得到里程计在里程计坐标系下的位姿odom_pose
  if(addScan(*scan, odom_pose))
  {
    ROS_DEBUG("scan processed");
    //mpose是权重最大的粒子位置
    GMapping::OrientedPoint mpose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;
    ROS_DEBUG("new best pose: %.3f %.3f %.3f", mpose.x, mpose.y, mpose.theta);
    ROS_DEBUG("odom pose: %.3f %.3f %.3f", odom_pose.x, odom_pose.y, odom_pose.theta);
    ROS_DEBUG("correction: %.3f %.3f %.3f", mpose.x - odom_pose.x, mpose.y - odom_pose.y, mpose.theta - odom_pose.theta);
	
    //此处的inverse是求整个变换的反变换
    //我觉得此处的命名不是很正确，不应该laser，应该是world
    tf::Transform laser_to_map = tf::Transform(tf::createQuaternionFromRPY(0, 0, mpose.theta), tf::Vector3(mpose.x, mpose.y, 0.0)).inverse();
    tf::Transform odom_to_laser = tf::Transform(tf::createQuaternionFromRPY(0, 0, odom_pose.theta), tf::Vector3(odom_pose.x, odom_pose.y, 0.0));

    map_to_odom_mutex_.lock();
    map_to_odom_ = (odom_to_laser * laser_to_map).inverse();//map坐标到odom坐标的变换，就只使用在了publishTransform函数中
    map_to_odom_mutex_.unlock();
	
   //如果没有获取到地图或者更新时间到了都要更新地图
   //updateMap函数中会got_map_=true
    if(!got_map_ || (scan->header.stamp - last_map_update) > map_update_interval_)
    {
	      updateMap(*scan);//!!!更新地图，非常重要的一个函数调用
	      last_map_update = scan->header.stamp;
	      ROS_DEBUG("Updated the map");
    }
  } else
    ROS_DEBUG("cannot process scan");
}

double
SlamGMapping::computePoseEntropy()
{
  double weight_total=0.0;
  for(std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
      it != gsp_->getParticles().end();
      ++it)
  {
    weight_total += it->weight;//将所有粒子的权重和累加
  }
  double entropy = 0.0;
  for(std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
      it != gsp_->getParticles().end();
      ++it)
  {
    if(it->weight/weight_total > 0.0)
      entropy += it->weight/weight_total * log(it->weight/weight_total);
  }
  return -entropy;
}

void
SlamGMapping::updateMap(const sensor_msgs::LaserScan& scan)
{
  ROS_DEBUG("Update map");
  boost::mutex::scoped_lock map_lock (map_mutex_);
  GMapping::ScanMatcher matcher;

  //激光传感器的参数有beam的数量，角度数据存放的地址，激光传感器的位置
  matcher.setLaserParameters(scan.ranges.size(), &(laser_angles_[0]),
                             gsp_laser_->getPose());

  matcher.setlaserMaxRange(maxRange_);
  matcher.setusableRange(maxUrange_);
  matcher.setgenerateMap(true);

  //搜索"int GridSlamProcessor::getBestParticleIndex()"就能找到getBestParticleIndex函数
  GMapping::GridSlamProcessor::Particle best =
          gsp_->getParticles()[gsp_->getBestParticleIndex()];//得到权重和最大的粒子序号
  std_msgs::Float64 entropy;
  entropy.data = computePoseEntropy();//这个参数是衡量机器人位置分布好坏的参数
  if(entropy.data > 0.0)
    entropy_publisher_.publish(entropy);

  if(!got_map_)//默认值是fasle，但是在这个函数的后半段会赋值为true，用于初始化
  {
	    map_.map.info.resolution = delta_;
	    map_.map.info.origin.position.x = 0.0;
	    map_.map.info.origin.position.y = 0.0;
	    map_.map.info.origin.position.z = 0.0;
	    map_.map.info.origin.orientation.x = 0.0;
	    map_.map.info.origin.orientation.y = 0.0;
	    map_.map.info.origin.orientation.z = 0.0;
	    map_.map.info.origin.orientation.w = 1.0;
  } 

  GMapping::Point center;
  //xmin_ ymin_ xmax_ ymax_用来初始化地图大小,对应的参数分别为xmin ymin xmax ymax，默认值分别为(-100,-100,100,100)
  //�(xmin_,ymin_)与(xmax_ ymax_)两个坐标点形成了世界坐标系下地图的左上角和右下角
  center.x=(xmin_ + xmax_) / 2.0;//默认的center.x=0,下面会对xmin_更新
  center.y=(ymin_ + ymax_) / 2.0;//默认的center.y=0
  
  //delta_为地图分辨率，对应的参数为delta，默认值是0.05(米)
  //搜索"real map"即可得到ScanMatcherMap的构造函数
  GMapping::ScanMatcherMap smap(center, xmin_, ymin_, xmax_, ymax_, 
                                delta_);

  ROS_DEBUG("Trajectory tree:");
  for(GMapping::GridSlamProcessor::TNode* n = best.node;n;n = n->parent)
  {
	    ROS_DEBUG("  %.3f %.3f %.3f",
	              n->pose.x,
	              n->pose.y,
	              n->pose.theta);
	    if(!n->reading)
	    {
	      ROS_DEBUG("Reading is NULL");
	      continue;
	    }
	    matcher.invalidateActiveArea();//m_activeAreaComputed=false;
	    matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
	    matcher.registerScan(smap, n->pose, &((*n->reading)[0]));//将扫描的特征注册到地图上
  }

  // the map may have expanded, so resize ros message as well
  //因为扫描获取的数据有可能使地图扩大了，因此需要重新调整地图的大小，这里的getMapSizeX会因为调用Map::resize而发生变化
  //而Map::grow从来没有被调用过，Map::resize在computeActiveArea(在ScanMatcher.cpp文件中)函数中被调用
  //因为经过粒子树更新地图之后，地图的大小发生了变化，因此需要重新更新地图的大小,更新map_的信息
  if(map_.map.info.width != (unsigned int) smap.getMapSizeX() || map_.map.info.height != (unsigned int) smap.getMapSizeY()) {

    // NOTE: The results of ScanMatcherMap::getSize() are different from the parameters given to the constructor
    //       so we must obtain the bounding box in a different way
    GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
    GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));//getMapSizeX返回值是m_mapSizeX
    xmin_ = wmin.x; ymin_ = wmin.y;//世界坐标系下地图左下角和右上角的坐标
    xmax_ = wmax.x; ymax_ = wmax.y;//这四个参数都是用来初始化地图大小的
    
    ROS_DEBUG("map size is now %dx%d pixels (%f,%f)-(%f, %f)", smap.getMapSizeX(), smap.getMapSizeY(),
              xmin_, ymin_, xmax_, ymax_);

    map_.map.info.width = smap.getMapSizeX();
    map_.map.info.height = smap.getMapSizeY();
    map_.map.info.origin.position.x = xmin_;
    map_.map.info.origin.position.y = ymin_;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);

    ROS_DEBUG("map origin: (%f, %f)", map_.map.info.origin.position.x, map_.map.info.origin.position.y);
  }

  //根据每个grid的概率然后设置一个阈值。更新地图
  for(int x=0; x < smap.getMapSizeX(); x++)
  {
	    for(int y=0; y < smap.getMapSizeY(); y++)
	    {
		      /// @todo Sort out the unknown vs. free vs. obstacle thresholding
		      GMapping::IntPoint p(x, y);
		      double occ=smap.cell(p);//看似简单的一句话，其实非常复杂，occ计算的是这个grid的透明度
		      assert(occ <= 1.0);//判断里面的表达式是否为真，如果不为真则终止程序的执行
		      if(occ < 0)
		        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;�//width*y+x
		      else if(occ > occ_thresh_)//occ_thresh_对应参数occ_thresh，Threshold on gmapping's occupancy values. Cells with greater occupancy are considered occupied，默认值0.25
		      {
		        //map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = (int)round(occ*100.0);
		        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;//width*y+x,如果大于透明度的阈值，则将其透明度设为100，认为是有障碍物
		      }
		      else//如果其透明度小于occ_thresh_大于0
		        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
	    }
  }
  got_map_ = true;

  //make sure to set the header information on the map
  map_.map.header.stamp = ros::Time::now();
  map_.map.header.frame_id = tf_.resolve( map_frame_ );
  // sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  // sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  sst_.publish(map_.map);//sst的定义:sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_.publish(map_.map.info);//这个是map_metadata  topic
}

//这个函数在启用dynamic_map service时才会调用
bool 
SlamGMapping::mapCallback(nav_msgs::GetMap::Request  &req,
                          nav_msgs::GetMap::Response &res)
{
  boost::mutex::scoped_lock map_lock (map_mutex_);
  if(got_map_ && map_.map.info.width && map_.map.info.height)
  {
    res = map_;
    return true;
  }
  else
    return false;
}

void SlamGMapping::publishTransform()
{
  map_to_odom_mutex_.lock();
  ros::Time tf_expiration = ros::Time::now() + ros::Duration(tf_delay_);
  //map_to_odom_为一个坐标系变换，子坐标系是odom_frame_，父坐标系是map_frame_
  //由此经过slam算法纠正过的小车的位置也发布到了tf树上了。
  //具体的map_to_odom_变换由laserCallback函数中得到，初始设置为0，
  tfB_->sendTransform( tf::StampedTransform (map_to_odom_, tf_expiration, map_frame_, odom_frame_));
  map_to_odom_mutex_.unlock();
}
