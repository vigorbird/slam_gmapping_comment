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
   //time(NULL)·µ»Ø´Ó1970ÄêÔªµ©ÎçÒ¹0µãµ½ÏÖÔÚµÄÃëÊı
   //Ö»ÔÚinitMapperÖĞ±»µ÷ÓÃ¹ı
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
  gsp_ = new GMapping::GridSlamProcessor(); //µ÷ÓÃÆä¹¹Ôìº¯Êı³õÊ¼»¯Èı¸ö²ÎÊı
  ROS_ASSERT(gsp_);//±í´ïÊ½½á¹ûÈç¹ûÎªfalse³ÌĞò½«Í£Ö¹Ö´ĞĞ

  tfB_ = new tf::TransformBroadcaster();//Õâ¸öÖ÷ÒªÓÃÀ´·¢²¼×ø±ê±ä»»
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
  map_update_interval_.fromSec(tmp);//½«ÉèÖÃµÄ²ÎÊı(µ¥Î»ÎªÃë)×ª»¯Îªros³ÖĞøÊ±¼ä
  
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
  //ÏÂÃæÎª·¢²¼µÄtopic
  entropy_publisher_ = private_nh_.advertise<std_msgs::Float64>("entropy", 1, true);
  sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  ss_ = node_.advertiseService("dynamic_map", &SlamGMapping::mapCallback, this);
  
  //¶©ÔÄscan topic£¬²¢ÇÒ½ÓÊÕµ½µÄÊı¾İÒª×ª»»µ½µÄÄ¿±ê×ø±êÏµÊÇodom_frame_
  //Ò»µ©tfÊ÷ÖĞÓĞodom_frame_µÄ×ø±êÏµĞÅÏ¢¾Íµ÷ÓÃº¯ÊılaserCallback
  scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "scan", 5);
  scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5);
  scan_filter_->registerCallback(boost::bind(&SlamGMapping::laserCallback, this, _1));//Ò»µ©Àï³Ì¼Æ×ø±êÏµ·¢²¼µ½tfÖĞÔòµ÷ÓÃ»Øµ÷º¯Êı

 //¿ªÆôÒ»¸öÏß³Ì
 //Ä¬ÈÏµÄtransform_publish_period_=0.05£¬
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

//transform_publish_periodÕâ¸ö²ÎÊıÊÇÈ«¾Ö²ÎÊı£¬´ÓlaunchÎÄ¼şÖĞ¶ÁÈ¡
void SlamGMapping::publishLoop(double transform_publish_period){
  if(transform_publish_period == 0)
    return;
  //ÉèÖÃrosÏµÍ³Ñ­»·µÄÆµÂÊ£¬µ¥Î»ÎªºÕ×È£¬ÏµÍ³Ä¬ÈÏµÄÆµÂÊÎª20Hz
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
	//½«×ø±êcentered_laser_pose_(¼¤¹â´«¸ĞÆ÷×ø±êÏµ)×ª»»Îªodom_frame×ø±êÏµÏÂµÄ×ø±ê£¬²¢½«×ø±ê±£´æÔÚodom_poseÖĞ
	//Ç°ÌáÊÇodom_frameÒÑ¾­ÓÉhandfreeÆ½Ì¨·¢²¼µ½ÁËtfÊ÷ÖĞ
	//centered_laser_pose_Ö»ÔÚinitMapperº¯ÊıÖĞµÃµ½£¬²¢ÇÒÊÇ³£Êı
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
  // Get the laser's pose, relative to base.ÈçºÎ»ñÈ¡µÄ¸ß¶ÈĞÅÏ¢????
  tf::Stamped<tf::Pose> ident;
  tf::Stamped<tf::Transform> laser_pose;
  ident.setIdentity();//set this transformation(±ä»») to identity(¼´µ¥Î»¾ØÕó)
  ident.frame_id_ = laser_frame_;
  ident.stamp_ = scan.header.stamp;//»ñÈ¡É¨ÃèÊı¾İµÄÊ±¼äĞÅÏ¢¡£stampÊı¾İÀàĞÍ:time stamp
  try
  {
    //³õÊ¼»¯Ê±ÏÈÓÃ»ù×ùµÄÎ»ÖÃĞÅÏ¢À´È·¶¨¼¤¹â´«¸ĞÆ÷µÄÎ»ÖÃ
    //»ù×ùµÄ×ø±êÏµĞÅÏ¢±ØĞëÊÂÏÈÒª·¢²¼µ½tfÊ÷ÖĞ
    //½«¼¤¹â×ø±êÏµÏÂµÄident×ø±ê×ª»»³Ébase×ø±êÏµµÄ×ø±ê£¬²¢½«½á¹û±£´æÔÚlaser_poseÖĞ
    //laser_poseÕâ¸ö²ÎÊıÖ»ÊÇÓÃÀ´ÅĞ¶Ï¼¤¹â´«¸ĞÆ÷ÓĞÃ»ÓĞ×°·´¡£
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
    //½«ÔÚbase×ø±êÏµÏÂµÄup×ø±ê×ª»»³Élaser_frame_×ø±êÏµÏÂ£¬²¢±£´æµ½up±äÁ¿
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
  //angle_minÊÇµÚÒ»¸öÉ¨ÃèµÄ½Ç¶ÈÊı¾İ£¬angle_maxÊÇ×îºóÒ»¸öÉ¨ÃèµÄ½Ç¶ÈÊı¾İ
  //Èç¹ûÎÒÃÇÊ¹ÓÃµÄÊÇrplidar£¬Ôòangle_min=0£¬angle_max=359*pi/180
  double angle_center = (scan.angle_min + scan.angle_max)/2;//¶ÔÓ¦180¶È

  if (up.z() > 0)//Ê²Ã´ÒâË¼??
  {
    do_reverse_range_ = scan.angle_min > scan.angle_max;
    //tf::Stamped<tf::Pose>½á¹¹ÖĞµÚÒ»¸ö²ÎÊıÉèÖÃyawµÄ½Ç¶È£¬µÚ¶ş¸ö²ÎÊıÊÇÎ»ÖÃ£¬µÚÈı¸ö²ÎÊıÊÇÊ±¼ä£¬µÚËÄ¸ö²ÎÊıÊÇ×ø±êÏµ
    //centered_laser_pose_ÊÇ¼¤¹â´«¸ĞÆ÷×ø±êÏµÏÂµÄ×ø±ê
    centered_laser_pose_ = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(0,0,angle_center),
                                                               tf::Vector3(0,0,0)), ros::Time::now(), laser_frame_);
    ROS_INFO("Laser is mounted upwards.");
  }
  else
  {
    do_reverse_range_ = scan.angle_min < scan.angle_max;
    centered_laser_pose_ = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(M_PI,0,-angle_center),//M_PIÊÇÔÚC++ÖĞ¶¨ÒåµÄ3.1415926
                                                               tf::Vector3(0,0,0)), ros::Time::now(), laser_frame_);
    ROS_INFO("Laser is mounted upside down.");
  }

 //************************************************************************
 //************************************************************************
 //************************************************************************
  // Compute the angles of the laser from -x to x, basically symmetric and in increasing order
  laser_angles_.resize(scan.ranges.size());
  // Make sure angles are started so that they are centered
  // ³õÊ¼»¯½Ç¶ÈÊı¾İ
  double theta = - std::fabs(scan.angle_min - scan.angle_max)/2;//Èç¹ûÎÒÃÇÊ¹ÓÃµÄrplidar£¬Ôòtheta¶ÔÓ¦180¶È
  for(unsigned int i=0; i<scan.ranges.size(); ++i)
  {
    laser_angles_[i]=theta;
    theta += std::fabs(scan.angle_increment);//Èç¹ûÎÒÃÇÊ¹ÓÃµÄÊÇriplidar£¬angle_increment¶ÔÓ¦1¶È
  }

  ROS_DEBUG("Laser angles in laser-frame: min: %.3f max: %.3f inc: %.3f", scan.angle_min, scan.angle_max,
            scan.angle_increment);
  ROS_DEBUG("Laser angles in top-down centered laser-frame: min: %.3f max: %.3f inc: %.3f", laser_angles_.front(),
            laser_angles_.back(), std::fabs(scan.angle_increment));

  //************************************************************************
 //************************************************************************
 //************************************************************************
  GMapping::OrientedPoint gmap_pose(0, 0, 0);//pose relative to the sensor center,Ç°Á½¸ö²ÎÊıÊÇÎ»ÖÃ£¬×îºóÒ»¸ö²ÎÊıÊÇ½Ç¶È

  ros::NodeHandle private_nh_("~");//Æô¶¯½Úµã£¬ÃüÃû¿Õ¼ä¾ÍÊÇ½ÚµãÃû³Æ
 // setting maxRange and maxUrange here so we can set a reasonable default 
  //maxRange_ÊÇ´«¸ĞÆ÷×î´óµÄ²âÁ¿¾àÀë£¬maxUrange_ÊÇ×î´óµÄ¿ÉÓÃµÄÓĞĞ§²âÁ¿¾àÀë
  //ros¹ÙÍøÉÏ½¨Òéset maxUrange < maximum range of the real sensor <= maxRange.
  if(!private_nh_.getParam("maxRange", maxRange_))
    maxRange_ = scan.range_max - 0.01;
  if(!private_nh_.getParam("maxUrange", maxUrange_))
    maxUrange_ = maxRange_;

  // The laser must be called "FLASER".Õâ¸öÊÇRangeSensorµÄÃû×Ö
  // We pass in the absolute value of the computed angle increment, on the
  // assumption that GMapping requires a positive angle increment.  If the
  // actual increment is negative, we'll swap the order of ranges before
  // feeding each scan to GMapping.
  //³õÊ¼»¯ÁË´«¸ĞÆ÷É¨ÃèµÄbeam½Ç¶ÈÊı¾İ(m_beam)ºÍ´Ë¿Ì´«¸ĞÆ÷µÄÎ»ÖÃ
  gsp_laser_ = new GMapping::RangeSensor("FLASER",
                                         gsp_laser_beam_count_,//gsp_laser_beam_count_ÓÃÀ´³õÊ¼»¯RangeSensorÖĞm_beamsÔªËØ¸öÊı
                                         fabs(scan.angle_increment),
                                         gmap_pose,//³õÊ¼»¯RangeSensor::m_pose£¬¼¤¹â´«¸ĞÆ÷ÓëĞ¡³µµ××ùµÄÏà¶ÔÎ»ÖÃĞÅÏ¢
                                         0.0,
                                         maxRange_);
  ROS_ASSERT(gsp_laser_);//ÉùÃ÷Õâ¸öÌá¹©µÄÃüÁî»òÕß²ÎÊıÊÇÕæ,·ñÔò³ÌĞò»áÖÕÖ¹Ö´ĞĞ


  //GMapping::SensorMap ÆäÊµÊÇstd::map<std::string, Sensor*>£¬
  //ÕâÀïµÄmap²»ÊÇ±íÊ¾µØÍ¼¶øÊÇÒ»ÖÖÌØÊâµÄÊı¾İ½á¹¹:¼üÖµ¶Ô
  GMapping::SensorMap smap;//ÕâÊÇÒ»¸ö¾Ö²¿±äÁ¿!!!!!!!!
  //make_pair¾ÍÊÇ¹¹ÔìÒ»¸ö¶ÔÏó£¬µÚÒ»¸öÔªËØÊÇgsp_laser_->getName()£¬µÚ¶ş¸öÔªËØÊÇgsp_laser_
  smap.insert(make_pair(gsp_laser_->getName(), gsp_laser_));
  gsp_->setSensorMap(smap);//Ö÷Òª×÷ÓÃÊÇ³õÊ¼»¯±äÁ¿m_matcher(ScanMatcher)ÖĞbeamµÄ½Ç¶È¡¢¸öÊıºÍ¼¤¹â´«¸ĞÆ÷´Ë¿ÌµÄÎ»ÖÃ

  gsp_odom_ = new GMapping::OdometrySensor(odom_frame_);
  ROS_ASSERT(gsp_odom_);


  /// @todo Expose setting an initial pose
  GMapping::OrientedPoint initialPose;
  //½«¼¤¹â×ø±êÏµÏÂµÄcentered_laser_pose×ø±ê£¬×ª»»³ÉÀï³Ì¼Æ×ø±êÏµÏÂµÄ×ø±ê£¬²¢±£´æÔÚinitialPoseÖĞ  
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
//ÉèÖÃm_matcher²ÎÊı£¬m_matcherÊÇGridSlamProcessorÀàÖĞ¶¨ÒåµÄ±äÁ¿
  gsp_->setMatchingParameters(maxUrange_, maxRange_, sigma_,
                              kernelSize_, lstep_, astep_, iterations_,
                              lsigma_, ogain_, lskip_);
//srr_:Odometry error in translation as a function of translation
//srt_:Odometry error in translation as a function of rotation (rho/theta)
//str_:Odometry error in rotation as a function of translation (theta/rho)
//stt_:Odometry error in rotation as a function of rotation (theta/theta)
//ÉèÖÃm_motionModel²ÎÊı£¬m_motionModelÊÇGridSlamProcessorÀàÖĞ¶¨ÒåµÄ±äÁ¿
  gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
//linearUpdate_:Process a scan each time the robot translates this far
//angularUpdate_:Process a scan each time the robot rotates this far
//resampleThreshold_:The Neff based resampling threshold
//ÉèÖÃm_linearThresholdDistance,m_angularThresholdDistance,m_resampleThreshold???ÕâĞ©²ÎÊıÃ»ÓĞÕÒµ½¶¨Òå
  gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
//Process a scan if the last scan processed is older than the update time in seconds.
//A value less than zero will turn time based updates off.
//ÉèÖÃperiod_£¬period_ÊÇGridSlamProcessorÀàÖĞ¶¨ÒåµÄ±äÁ¿
  gsp_->setUpdatePeriod(temporalUpdate_);

  //!!!!!!!Õâ¸öÒªÓÈÆä×¢Òâ!!!!ºóÃæ»áÓÃµ½!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //½«GridSlamProcessorÀàÖĞµÄm_generateMap=false
  gsp_->setgenerateMap(false);
  
  //particles_:Number of particles in the filter
  //xmin_,ymin_,xmax_, ymax_:Initial map size
  //delta_:Resolution of the map
  
  //!!!!Õâ¸öº¯ÊıÒªÖ÷Òª·ÖÎö!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //ËÑË÷"void GridSlamProcessor::init"¾ÍÄÜÕÒµ½¶ÔÓ¦º¯Êı
  //initialPoseÊÇÀï³Ì¼Æ×ø±êÏµÏÂµÄ¼¤¹â´«¸ĞÆ÷µÄ×ø±ê
  //Ä¬ÈÏÖµ:xmin_=-100,xmax=100,particles_=30,delta_=0.05
  gsp_->GridSlamProcessor::init(particles_, xmin_, ymin_, xmax_, ymax_,
                                delta_, initialPose);
  
  //************************************************************************
 //************************************************************************
  //llsamplerange_:Translational sampling range for the likelihood
  //ÏÂÃæµÄ¼¸ĞĞ´úÂë¶¼ÊÇÊ¹ÓÃºê¶¨ÒåµÄsetllsamplerangeµÈº¯Êı
  //¶¼ÊÇ¶Ôm_matcherÖĞµÄ²ÎÊı½øĞĞÉèÖÃ
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
  //Ö®Ç°ÔÚ¹¹ÔìSlamGMappingÊ±¶¨ÒåµÄseed_ = time(NULL)£¬
  //±¾ÖÊÉÏÊÇµ÷ÓÃÁËsrand48º¯Êı,²¢½«µÚÒ»¸ö²ÎÊı×÷Îª·½²î(ÆÚÍûÄ¬ÈÏÎª0)£¬Éú³ÉÒ»¸ö·ş´Ó¸ßË¹·Ö²¼µÄËæ»úÊı
  //×¢Òâ±¾À´Õâ¸öº¯ÊıÊÇÓĞ·µ»ØÖµµÄ£¬·µ»ØÖµ¾ÍÊÇ·ş´Ó¸ßË¹µÄËæ»úÊı£¬µ«ÊÇ´Ë´¦µÄÄ¿µÄÖ÷ÒªÊÇ³õÊ¼»¯Ëæ»úĞòÁĞ
  GMapping::sampleGaussian(1,seed_);
  
  ROS_INFO("Initialization complete");

  return true;
}


bool
SlamGMapping::addScan(const sensor_msgs::LaserScan& scan, GMapping::OrientedPoint& gmap_pose)
{
  if(!getOdomPose(gmap_pose, scan.header.stamp))//¸ù¾İÀï³Ì¼ÆĞÅÏ¢µÃµ½Àï³Ì¼ÆÔÚÊÀ½ç×ø±êÏµÏÂµÄ×ø±êgmap_pose
     return false;
  
  if(scan.ranges.size() != gsp_laser_beam_count_)
    return false;

  // GMapping wants an array of doubles...
  double* ranges_double = new double[scan.ranges.size()];
  // If the angle increment is negative, we have to invert the order of the readings.
  if (do_reverse_range_)//´æ´¢µÄÉ¨ÃèÊı¾İÒ»¶¨ÊÇ´Ó´óµ½Ğ¡
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
  else//Èç¹ûÎÒÃÇÊ¹ÓÃµÄÊÇrplidar£¬Ôò½øÈëÕâ¸öÌõ¼şÅĞ¶Ï 
  {
	    for(unsigned int i=0; i < scan.ranges.size(); i++)
	    {
	      // Must filter out short readings, because the mapper won't
	      //Èç¹û»ñµÃµÄ¼¤¹â¾àÀëÊı¾İĞ¡ÓÚÉè¶¨µÄÔÊĞí×îĞ¡ÖµÔò½«ÆäÖµ¸³ÖµÎª×î´óÖµ
	      if(scan.ranges[i] < scan.range_min)
	        ranges_double[i] = (double)scan.range_max;
	      else
	        ranges_double[i] = (double)scan.ranges[i];
	    }
  }
 
  GMapping::RangeReading reading(scan.ranges.size(),
                                 ranges_double,
                                 gsp_laser_,//Õâ¸ö²ÎÊıÔÚinitMapperÖĞÉú³É
                                 scan.header.stamp.toSec());

  // ...but it deep copies them in RangeReading constructor, so we don't
  // need to keep our array around.
  delete[] ranges_double;


  //gmap_poseÕâ¸ö²ÎÊıÊÇÓÉº¯Êı¸Õ¿ªÊ¼Ê±getOdomPoseÕâ¸öº¯ÊıµÃµ½
  //Àï³Ì¼ÆÔÚÊÀ½ç×ø±êÏµÏÂµÄ×ø±ê
  reading.setPose(gmap_pose);//Õâ¸öÎ»ÖÃĞÅÏ¢ÊÇÓÉÉÏÃæµÄgetOdomPoseº¯ÊıµÃµ½µÄ

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


//Ò»µ©Àï³Ì¼ÆµÄÎ»ÖÃ·¢²¼µ½tfÊ÷ºó×Ô¶¯µ÷ÓÃÕâ¸öº¯Êı£¬ÆäÊäÈëµÄ²ÎÊıÊÇ¼¤¹âÉ¨ÃèµÄÊı¾İ
void
SlamGMapping::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  laser_count_++;
  if ((laser_count_ % throttle_scans_) != 0)//throttle_scans_Ä¬ÈÏÖµ=1
    return;

  static ros::Time last_map_update(0,0);//µÚÒ»¸öµ¥Î»ÊÇÃë µÚ¶ş¸öµ¥Î»ÊÇÄÉÃë

  // We can't initialize the mapper until we've got the first scan
  if(!got_first_scan_)
  {
    if(!initMapper(*scan))
      return;
    got_first_scan_ = true;
  }

  GMapping::OrientedPoint odom_pose;
  
  //addScanº¯ÊıµÄÖ÷Òª×÷ÓÃÊÇ¶ÁÈ¡scanµÄÊı¾İ²¢µÃµ½Àï³Ì¼ÆÔÚÀï³Ì¼Æ×ø±êÏµÏÂµÄÎ»×Ëodom_pose
  if(addScan(*scan, odom_pose))
  {
    ROS_DEBUG("scan processed");
    //mposeÊÇÈ¨ÖØ×î´óµÄÁ£×ÓÎ»ÖÃ
    GMapping::OrientedPoint mpose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;
    ROS_DEBUG("new best pose: %.3f %.3f %.3f", mpose.x, mpose.y, mpose.theta);
    ROS_DEBUG("odom pose: %.3f %.3f %.3f", odom_pose.x, odom_pose.y, odom_pose.theta);
    ROS_DEBUG("correction: %.3f %.3f %.3f", mpose.x - odom_pose.x, mpose.y - odom_pose.y, mpose.theta - odom_pose.theta);
	
    //´Ë´¦µÄinverseÊÇÇóÕû¸ö±ä»»µÄ·´±ä»»
    //ÎÒ¾õµÃ´Ë´¦µÄÃüÃû²»ÊÇºÜÕıÈ·£¬²»Ó¦¸Ãlaser£¬Ó¦¸ÃÊÇworld
    tf::Transform laser_to_map = tf::Transform(tf::createQuaternionFromRPY(0, 0, mpose.theta), tf::Vector3(mpose.x, mpose.y, 0.0)).inverse();
    tf::Transform odom_to_laser = tf::Transform(tf::createQuaternionFromRPY(0, 0, odom_pose.theta), tf::Vector3(odom_pose.x, odom_pose.y, 0.0));

    map_to_odom_mutex_.lock();
    map_to_odom_ = (odom_to_laser * laser_to_map).inverse();//map×ø±êµ½odom×ø±êµÄ±ä»»£¬¾ÍÖ»Ê¹ÓÃÔÚÁËpublishTransformº¯ÊıÖĞ
    map_to_odom_mutex_.unlock();
	
   //Èç¹ûÃ»ÓĞ»ñÈ¡µ½µØÍ¼»òÕß¸üĞÂÊ±¼äµ½ÁË¶¼Òª¸üĞÂµØÍ¼
   //updateMapº¯ÊıÖĞ»ágot_map_=true
    if(!got_map_ || (scan->header.stamp - last_map_update) > map_update_interval_)
    {
	      updateMap(*scan);//!!!¸üĞÂµØÍ¼£¬·Ç³£ÖØÒªµÄÒ»¸öº¯Êıµ÷ÓÃ
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
    weight_total += it->weight;//½«ËùÓĞÁ£×ÓµÄÈ¨ÖØºÍÀÛ¼Ó
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

  //¼¤¹â´«¸ĞÆ÷µÄ²ÎÊıÓĞbeamµÄÊıÁ¿£¬½Ç¶ÈÊı¾İ´æ·ÅµÄµØÖ·£¬¼¤¹â´«¸ĞÆ÷µÄÎ»ÖÃ
  matcher.setLaserParameters(scan.ranges.size(), &(laser_angles_[0]),
                             gsp_laser_->getPose());

  matcher.setlaserMaxRange(maxRange_);
  matcher.setusableRange(maxUrange_);
  matcher.setgenerateMap(true);

  //ËÑË÷"int GridSlamProcessor::getBestParticleIndex()"¾ÍÄÜÕÒµ½getBestParticleIndexº¯Êı
  GMapping::GridSlamProcessor::Particle best =
          gsp_->getParticles()[gsp_->getBestParticleIndex()];//µÃµ½È¨ÖØºÍ×î´óµÄÁ£×ÓĞòºÅ
  std_msgs::Float64 entropy;
  entropy.data = computePoseEntropy();//Õâ¸ö²ÎÊıÊÇºâÁ¿»úÆ÷ÈËÎ»ÖÃ·Ö²¼ºÃ»µµÄ²ÎÊı
  if(entropy.data > 0.0)
    entropy_publisher_.publish(entropy);

  if(!got_map_)//Ä¬ÈÏÖµÊÇfasle£¬µ«ÊÇÔÚÕâ¸öº¯ÊıµÄºó°ë¶Î»á¸³ÖµÎªtrue£¬ÓÃÓÚ³õÊ¼»¯
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
  //xmin_ ymin_ xmax_ ymax_ÓÃÀ´³õÊ¼»¯µØÍ¼´óĞ¡,¶ÔÓ¦µÄ²ÎÊı·Ö±ğÎªxmin ymin xmax ymax£¬Ä¬ÈÏÖµ·Ö±ğÎª(-100,-100,100,100)
  //Î(xmin_,ymin_)Óë(xmax_ ymax_)Á½¸ö×ø±êµãĞÎ³ÉÁËÊÀ½ç×ø±êÏµÏÂµØÍ¼µÄ×óÉÏ½ÇºÍÓÒÏÂ½Ç
  center.x=(xmin_ + xmax_) / 2.0;//Ä¬ÈÏµÄcenter.x=0,ÏÂÃæ»á¶Ôxmin_¸üĞÂ
  center.y=(ymin_ + ymax_) / 2.0;//Ä¬ÈÏµÄcenter.y=0
  
  //delta_ÎªµØÍ¼·Ö±æÂÊ£¬¶ÔÓ¦µÄ²ÎÊıÎªdelta£¬Ä¬ÈÏÖµÊÇ0.05(Ã×)
  //ËÑË÷"real map"¼´¿ÉµÃµ½ScanMatcherMapµÄ¹¹Ôìº¯Êı
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
	    matcher.registerScan(smap, n->pose, &((*n->reading)[0]));//½«É¨ÃèµÄÌØÕ÷×¢²áµ½µØÍ¼ÉÏ
  }

  // the map may have expanded, so resize ros message as well
  //ÒòÎªÉ¨Ãè»ñÈ¡µÄÊı¾İÓĞ¿ÉÄÜÊ¹µØÍ¼À©´óÁË£¬Òò´ËĞèÒªÖØĞÂµ÷ÕûµØÍ¼µÄ´óĞ¡£¬ÕâÀïµÄgetMapSizeX»áÒòÎªµ÷ÓÃMap::resize¶ø·¢Éú±ä»¯
  //¶øMap::grow´ÓÀ´Ã»ÓĞ±»µ÷ÓÃ¹ı£¬Map::resizeÔÚcomputeActiveArea(ÔÚScanMatcher.cppÎÄ¼şÖĞ)º¯ÊıÖĞ±»µ÷ÓÃ
  //ÒòÎª¾­¹ıÁ£×ÓÊ÷¸üĞÂµØÍ¼Ö®ºó£¬µØÍ¼µÄ´óĞ¡·¢ÉúÁË±ä»¯£¬Òò´ËĞèÒªÖØĞÂ¸üĞÂµØÍ¼µÄ´óĞ¡,¸üĞÂmap_µÄĞÅÏ¢
  if(map_.map.info.width != (unsigned int) smap.getMapSizeX() || map_.map.info.height != (unsigned int) smap.getMapSizeY()) {

    // NOTE: The results of ScanMatcherMap::getSize() are different from the parameters given to the constructor
    //       so we must obtain the bounding box in a different way
    GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
    GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));//getMapSizeX·µ»ØÖµÊÇm_mapSizeX
    xmin_ = wmin.x; ymin_ = wmin.y;//ÊÀ½ç×ø±êÏµÏÂµØÍ¼×óÏÂ½ÇºÍÓÒÉÏ½ÇµÄ×ø±ê
    xmax_ = wmax.x; ymax_ = wmax.y;//ÕâËÄ¸ö²ÎÊı¶¼ÊÇÓÃÀ´³õÊ¼»¯µØÍ¼´óĞ¡µÄ
    
    ROS_DEBUG("map size is now %dx%d pixels (%f,%f)-(%f, %f)", smap.getMapSizeX(), smap.getMapSizeY(),
              xmin_, ymin_, xmax_, ymax_);

    map_.map.info.width = smap.getMapSizeX();
    map_.map.info.height = smap.getMapSizeY();
    map_.map.info.origin.position.x = xmin_;
    map_.map.info.origin.position.y = ymin_;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);

    ROS_DEBUG("map origin: (%f, %f)", map_.map.info.origin.position.x, map_.map.info.origin.position.y);
  }

  //¸ù¾İÃ¿¸ögridµÄ¸ÅÂÊÈ»ºóÉèÖÃÒ»¸öãĞÖµ¡£¸üĞÂµØÍ¼
  for(int x=0; x < smap.getMapSizeX(); x++)
  {
	    for(int y=0; y < smap.getMapSizeY(); y++)
	    {
		      /// @todo Sort out the unknown vs. free vs. obstacle thresholding
		      GMapping::IntPoint p(x, y);
		      double occ=smap.cell(p);//¿´ËÆ¼òµ¥µÄÒ»¾ä»°£¬ÆäÊµ·Ç³£¸´ÔÓ£¬occ¼ÆËãµÄÊÇÕâ¸ögridµÄÍ¸Ã÷¶È
		      assert(occ <= 1.0);//ÅĞ¶ÏÀïÃæµÄ±í´ïÊ½ÊÇ·ñÎªÕæ£¬Èç¹û²»ÎªÕæÔòÖÕÖ¹³ÌĞòµÄÖ´ĞĞ
		      if(occ < 0)
		        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;¡//width*y+x
		      else if(occ > occ_thresh_)//occ_thresh_¶ÔÓ¦²ÎÊıocc_thresh£¬Threshold on gmapping's occupancy values. Cells with greater occupancy are considered occupied£¬Ä¬ÈÏÖµ0.25
		      {
		        //map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = (int)round(occ*100.0);
		        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;//width*y+x,Èç¹û´óÓÚÍ¸Ã÷¶ÈµÄãĞÖµ£¬Ôò½«ÆäÍ¸Ã÷¶ÈÉèÎª100£¬ÈÏÎªÊÇÓĞÕÏ°­Îï
		      }
		      else//Èç¹ûÆäÍ¸Ã÷¶ÈĞ¡ÓÚocc_thresh_´óÓÚ0
		        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
	    }
  }
  got_map_ = true;

  //make sure to set the header information on the map
  map_.map.header.stamp = ros::Time::now();
  map_.map.header.frame_id = tf_.resolve( map_frame_ );
  // sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  // sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  sst_.publish(map_.map);//sstµÄ¶¨Òå:sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_.publish(map_.map.info);//Õâ¸öÊÇmap_metadata  topic
}

//Õâ¸öº¯ÊıÔÚÆôÓÃdynamic_map serviceÊ±²Å»áµ÷ÓÃ
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
  //map_to_odom_ÎªÒ»¸ö×ø±êÏµ±ä»»£¬×Ó×ø±êÏµÊÇodom_frame_£¬¸¸×ø±êÏµÊÇmap_frame_
  //ÓÉ´Ë¾­¹ıslamËã·¨¾ÀÕı¹ıµÄĞ¡³µµÄÎ»ÖÃÒ²·¢²¼µ½ÁËtfÊ÷ÉÏÁË¡£
  //¾ßÌåµÄmap_to_odom_±ä»»ÓÉlaserCallbackº¯ÊıÖĞµÃµ½£¬³õÊ¼ÉèÖÃÎª0£¬
  tfB_->sendTransform( tf::StampedTransform (map_to_odom_, tf_expiration, map_frame_, odom_frame_));
  map_to_odom_mutex_.unlock();
}
