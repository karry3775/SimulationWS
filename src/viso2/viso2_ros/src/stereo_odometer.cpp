#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/stereo_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <nav_msgs/Odometry.h> // for odom message
#include <geometry_msgs/Twist.h> // for cmd_vel
#include <sensor_msgs/Imu.h> // for imu readings
#include <tf/transform_datatypes.h> // for quat euler conversions
#include <math.h>

#include <viso_stereo.h>

#include <viso2_ros/VisoInfo.h>

#include "stereo_processor.h"
#include "odometer_base.h"
#include "odometry_params.h"

#include <iostream> // for i/o
// #include <fstream> // for file handling

// to remove after debugging
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/subscriber.h>

namespace viso2_ros
{

// some arbitrary values (0.1m^2 linear cov. 10deg^2. angular cov.)
static const boost::array<double, 36> STANDARD_POSE_COVARIANCE =
{ { 0.1, 0, 0, 0, 0, 0,
    0, 0.1, 0, 0, 0, 0,
    0, 0, 0.1, 0, 0, 0,
    0, 0, 0, 0.17, 0, 0,
    0, 0, 0, 0, 0.17, 0,
    0, 0, 0, 0, 0, 0.17 } };
static const boost::array<double, 36> STANDARD_TWIST_COVARIANCE =
{ { 0.002, 0, 0, 0, 0, 0,
    0, 0.002, 0, 0, 0, 0,
    0, 0, 0.05, 0, 0, 0,
    0, 0, 0, 0.09, 0, 0,
    0, 0, 0, 0, 0.09, 0,
    0, 0, 0, 0, 0, 0.09 } };
static const boost::array<double, 36> BAD_COVARIANCE =
{ { 9999, 0, 0, 0, 0, 0,
    0, 9999, 0, 0, 0, 0,
    0, 0, 9999, 0, 0, 0,
    0, 0, 0, 9999, 0, 0,
    0, 0, 0, 0, 9999, 0,
    0, 0, 0, 0, 0, 9999 } };


class StereoOdometer : public StereoProcessor, public OdometerBase
{

private:

  boost::shared_ptr<VisualOdometryStereo> visual_odometer_;
  VisualOdometryStereo::parameters visual_odometer_params_;

  ros::Publisher point_cloud_pub_;
  ros::Publisher info_pub_;

  bool got_lost_;

  // change reference frame method. 0, 1 or 2. 0 means allways change. 1 and 2 explained below
  int ref_frame_change_method_;
  bool change_reference_frame_;
  double ref_frame_motion_threshold_; // method 1. Change the reference frame if last motion is small
  int ref_frame_inlier_threshold_; // method 2. Change the reference frame if the number of inliers is low
  Matrix reference_motion_;
  double ref_yaw_; // extra variable to track motion
  double prev_time_; // for finding the delta for cmd_vel
  // file variable
  std::ofstream outfile;

public:

  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

  StereoOdometer(const std::string& transport) :
    StereoProcessor(transport), OdometerBase(),
    got_lost_(false), change_reference_frame_(false)
  {
    // Read local parameters
    ros::NodeHandle local_nh("~");
    odometry_params::loadParams(local_nh, visual_odometer_params_);

    local_nh.param("ref_frame_change_method", ref_frame_change_method_, 0);
    local_nh.param("ref_frame_motion_threshold", ref_frame_motion_threshold_, 5.0);
    local_nh.param("ref_frame_inlier_threshold", ref_frame_inlier_threshold_, 150);

    point_cloud_pub_ = local_nh.advertise<PointCloud>("point_cloud", 1);
    info_pub_ = local_nh.advertise<VisoInfo>("info", 1);

    reference_motion_ = Matrix::eye(4);

    // open the file
    // outfile.open("pose_data.txt");
    // create a subscriber for odomCallback
    //ros::Subscriber odom_sub = local_nh.subscribe<nav_msgs::Odometry>("/ground_truth/state", 1000, boost::bind(&StereoOdometer::odomCallback, this,_1));

  }

  ~StereoOdometer(){
    // outfile.close();
  }


protected:

  void initOdometer(
      const sensor_msgs::CameraInfoConstPtr& l_info_msg,
      const sensor_msgs::CameraInfoConstPtr& r_info_msg)
  {
    int queue_size;
    bool approximate_sync;
    ros::NodeHandle local_nh("~");
    local_nh.param("queue_size", queue_size, 5);
    local_nh.param("approximate_sync", approximate_sync, false);

    // read calibration info from camera info message
    // to fill remaining parameters
    image_geometry::StereoCameraModel model;
    model.fromCameraInfo(*l_info_msg, *r_info_msg);
    visual_odometer_params_.base      = model.baseline();
    visual_odometer_params_.calib.cu  = model.left().cx();
    visual_odometer_params_.calib.cv  = model.left().cy();
    visual_odometer_params_.calib.f   = model.left().fx();

    visual_odometer_.reset(new VisualOdometryStereo(visual_odometer_params_));
    if (l_info_msg->header.frame_id != "") setSensorFrameId(l_info_msg->header.frame_id);
    ROS_INFO_STREAM("Initialized libviso2 stereo odometry "
                    "with the following parameters:" << std::endl <<
                    visual_odometer_params_ <<
                    "  queue_size = " << queue_size << std::endl <<
                    "  approximate_sync = " << approximate_sync << std::endl <<
                    "  ref_frame_change_method = " << ref_frame_change_method_ << std::endl <<
                    "  ref_frame_motion_threshold = " << ref_frame_motion_threshold_ << std::endl <<
                    "  ref_frame_inlier_threshold = " << ref_frame_inlier_threshold_);
  }

  // create an odomCallback function here
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
    // first we will just print the message out
    // ROS_INFO_STREAM("odom callback was called successfully" << std::endl);
    // // we need to somehow check the contents of tf::Transform
    // // get the yaw value from the odom_msg
    // setRotationForIntegratedPose(odom_msg);

  }

  void imageCallback(
      const sensor_msgs::ImageConstPtr& l_image_msg,
      const sensor_msgs::ImageConstPtr& r_image_msg,
      const sensor_msgs::CameraInfoConstPtr& l_info_msg,
      const sensor_msgs::CameraInfoConstPtr& r_info_msg)
  {
    ros::WallTime start_time = ros::WallTime::now();
    bool first_run = false;
    // create odometer if not exists
    if (!visual_odometer_)
    {
      first_run = true;
      initOdometer(l_info_msg, r_info_msg);
      prev_time_ = ros::WallTime::now().toSec();
    }

    // convert images if necessary
    uint8_t *l_image_data, *r_image_data;
    int l_step, r_step;
    cv_bridge::CvImageConstPtr l_cv_ptr, r_cv_ptr;
    l_cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8);
    l_image_data = l_cv_ptr->image.data;
    l_step = l_cv_ptr->image.step[0];
    r_cv_ptr = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8);
    r_image_data = r_cv_ptr->image.data;
    r_step = r_cv_ptr->image.step[0];

    ROS_ASSERT(l_step == r_step);
    ROS_ASSERT(l_image_msg->width == r_image_msg->width);
    ROS_ASSERT(l_image_msg->height == r_image_msg->height);

    int32_t dims[] = {l_image_msg->width, l_image_msg->height, l_step};
    // on first run or when odometer got lost, only feed the odometer with
    // images without retrieving data
    if (first_run || got_lost_)
    {
      visual_odometer_->process(l_image_data, r_image_data, dims);
      got_lost_ = false;
      // on first run publish zero once
      if (first_run)
      {
        tf::Transform delta_transform;
        delta_transform.setIdentity();
        integrateAndPublish(delta_transform, l_image_msg->header.stamp);
        ref_yaw_ = 0.0;
      }
    }
    else
    {
      bool success = visual_odometer_->process(
          l_image_data, r_image_data, dims, change_reference_frame_);
      if (success)
      {
        Matrix motion = Matrix::inv(visual_odometer_->getMotion());

        // I need to get this motion parameter
        ROS_INFO_STREAM("libviso2 returned the following motion:\n" << motion);

        ROS_DEBUG("Found %i matches with %i inliers.",
                  visual_odometer_->getNumberOfMatches(),
                  visual_odometer_->getNumberOfInliers());
        ROS_DEBUG_STREAM("libviso2 returned the following motion:\n" << motion);
        Matrix camera_motion;
        // if image was replaced due to small motion we have to subtract the
        // last motion to get the increment
        if (change_reference_frame_)
        {
          camera_motion = Matrix::inv(reference_motion_) * motion;
        }
        else
        {
          // image was not replaced, report full motion from odometer
          camera_motion = motion;
        }
        reference_motion_ = motion; // store last motion as reference



        // before passing the values, I need to do two things
        // get the ground truth yaw and formulate the rot_mat in such a way that is consistent with the
        camera_motion.val[1][3] = 0.0;
        camera_motion.val[0][3] = 0.0;

        // now lets subscribe to the odom frame
        ros::NodeHandle local_nh("~");

        // lets wait for jacky/cmd_vel (geometry_msgs::Twist)
        boost::shared_ptr<geometry_msgs::Twist const> shared_cmd_vel_ptr;
        geometry_msgs::Twist cmd_vel_msg;
        shared_cmd_vel_ptr = ros::topic::waitForMessage<geometry_msgs::Twist>("/jacky/cmd_vel", local_nh);
        if(shared_cmd_vel_ptr != NULL){
          cmd_vel_msg = *shared_cmd_vel_ptr;
        }
        // find delta_x using just the cmd_vel

        // first get the current time
        double cur_time = ros::WallTime::now().toSec();
        double delta_time_cmd = cur_time - prev_time_;
        prev_time_ = cur_time;

        double delta_x_cmd = cmd_vel_msg.linear.x * delta_time_cmd;

        // lets wait for imu messages
        // boost::shared_ptr<sensor_msgs::Imu const> shared_imu_ptr;
        // sensor_msgs::Imu imu_msg;
        // shared_imu_ptr = ros::topic::waitForMessage<sensor_msgs::Imu>("/imu", local_nh);
        //
        // if(shared_imu_ptr != NULL){
        //   imu_msg = *shared_imu_ptr;
        // }

        if(cmd_vel_msg.linear.x == 0){
          // this means we are definetely at rest
          // trust the cmd_vel_msg
          camera_motion.val[2][3] = 0.0;
        }
        else{
          // if moving then simply
          camera_motion.val[2][3] = 0.80 * delta_x_cmd + 0.20 * camera_motion.val[2][3];
        }

        boost::shared_ptr<nav_msgs::Odometry const> shared_odom_msg_ptr;
        nav_msgs::Odometry odom_msg;
        // most likely this waiting for ground truth might be causing it to differ
        // lets actually check how much will the thing have to wait
        // if its not that much then this possibly can be the issue
        shared_odom_msg_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("/ground_truth/state",local_nh);
        if(shared_odom_msg_ptr != NULL){
          odom_msg = *shared_odom_msg_ptr;
        }

        tf::Quaternion q(odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        double cos_tht = cos(-(yaw-ref_yaw_));
        double sin_tht = sin(-(yaw-ref_yaw_));

        // storing previous yaw value
        ref_yaw_ = yaw;

        // tf::Matrix3x3 rot_mat(
        //   camera_motion.val[0][0], camera_motion.val[0][1], camera_motion.val[0][2],
        //   camera_motion.val[1][0], camera_motion.val[1][1], camera_motion.val[1][2],
        //   camera_motion.val[2][0], camera_motion.val[2][1], camera_motion.val[2][2]);
        tf::Matrix3x3 rot_mat(cos_tht , 0, sin_tht,
                              0       , 1,       0,
                              -sin_tht, 0, cos_tht);
        tf::Vector3 t(camera_motion.val[0][3], camera_motion.val[1][3], camera_motion.val[2][3]);
        tf::Transform delta_transform(rot_mat, t);

        // RECORD DATA INTO A FILE
        // store yaw information
        // store delta_x, delta_z (which will map to -delta_y and delta_x)
      //  double delta_x = camera_motion.val[2][3];
      //  double delta_y = -camera_motion.val[0][3];
      //  outfile << delta_x << "," << delta_y << "," << yaw << "," << odom_msg.pose.pose.position.x << "," << odom_msg.pose.pose.position.y << std::endl;

        setPoseCovariance(STANDARD_POSE_COVARIANCE);
        setTwistCovariance(STANDARD_TWIST_COVARIANCE);

        integrateAndPublish(delta_transform, l_image_msg->header.stamp);

        if (point_cloud_pub_.getNumSubscribers() > 0)
        {
          computeAndPublishPointCloud(l_info_msg, l_image_msg, r_info_msg,
                                      visual_odometer_->getMatches(),
                                      visual_odometer_->getInlierIndices());
        }
      }
      else
      {
        setPoseCovariance(BAD_COVARIANCE);
        setTwistCovariance(BAD_COVARIANCE);
        tf::Transform delta_transform;
        delta_transform.setIdentity();
        integrateAndPublish(delta_transform, l_image_msg->header.stamp);

        ROS_DEBUG("Call to VisualOdometryStereo::process() failed.");
        ROS_WARN_THROTTLE(10.0, "Visual Odometer got lost!");
        got_lost_ = true;
      }

      if(success)
      {

        // Proceed depending on the reference frame change method
        switch ( ref_frame_change_method_ )
        {
          case 1:
          {
            // calculate current feature flow
            double feature_flow = computeFeatureFlow(visual_odometer_->getMatches());
            change_reference_frame_ = (feature_flow < ref_frame_motion_threshold_);
            ROS_DEBUG_STREAM("Feature flow is " << feature_flow
                << ", marking last motion as "
                << (change_reference_frame_ ? "small." : "normal."));
            break;
          }
          case 2:
          {
            change_reference_frame_ = (visual_odometer_->getNumberOfInliers() > ref_frame_inlier_threshold_);
            break;
          }
          default:
            change_reference_frame_ = false;
        }

      }
      else
        change_reference_frame_ = false;

      if(!change_reference_frame_)
        ROS_DEBUG_STREAM("Changing reference frame");

      // create and publish viso2 info msg
      VisoInfo info_msg;
      info_msg.header.stamp = l_image_msg->header.stamp;
      info_msg.got_lost = !success;
      info_msg.change_reference_frame = !change_reference_frame_;
      info_msg.num_matches = visual_odometer_->getNumberOfMatches();
      info_msg.num_inliers = visual_odometer_->getNumberOfInliers();
      ros::WallDuration time_elapsed = ros::WallTime::now() - start_time;
      info_msg.runtime = time_elapsed.toSec();
      info_pub_.publish(info_msg);
    }
  }

  double computeFeatureFlow(
      const std::vector<Matcher::p_match>& matches)
  {
    double total_flow = 0.0;
    for (size_t i = 0; i < matches.size(); ++i)
    {
      double x_diff = matches[i].u1c - matches[i].u1p;
      double y_diff = matches[i].v1c - matches[i].v1p;
      total_flow += sqrt(x_diff * x_diff + y_diff * y_diff);
    }
    return total_flow / matches.size();
  }

  void computeAndPublishPointCloud(
      const sensor_msgs::CameraInfoConstPtr& l_info_msg,
      const sensor_msgs::ImageConstPtr& l_image_msg,
      const sensor_msgs::CameraInfoConstPtr& r_info_msg,
      const std::vector<Matcher::p_match>& matches,
      const std::vector<int32_t>& inlier_indices)
  {
    try
    {
      cv_bridge::CvImageConstPtr cv_ptr;
      cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::RGB8);
      // read calibration info from camera info message
      image_geometry::StereoCameraModel model;
      model.fromCameraInfo(*l_info_msg, *r_info_msg);
      PointCloud::Ptr point_cloud(new PointCloud());
      point_cloud->header.frame_id = getSensorFrameId();
      point_cloud->header.stamp = pcl_conversions::toPCL(l_info_msg->header).stamp;
      point_cloud->width = 1;
      point_cloud->height = inlier_indices.size();
      point_cloud->points.resize(inlier_indices.size());

      for (size_t i = 0; i < inlier_indices.size(); ++i)
      {
        const Matcher::p_match& match = matches[inlier_indices[i]];
        cv::Point2d left_uv;
        left_uv.x = match.u1c;
        left_uv.y = match.v1c;
        cv::Point3d point;
        double disparity = match.u1c - match.u2c;
        model.projectDisparityTo3d(left_uv, disparity, point);
        point_cloud->points[i].x = point.x;
        point_cloud->points[i].y = point.y;
        point_cloud->points[i].z = point.z;
        cv::Vec3b color = cv_ptr->image.at<cv::Vec3b>(left_uv.y,left_uv.x);
        point_cloud->points[i].r = color[0];
        point_cloud->points[i].g = color[1];
        point_cloud->points[i].b = color[2];
      }
      ROS_DEBUG("Publishing point cloud with %zu points.", point_cloud->size());
      point_cloud_pub_.publish(point_cloud);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }


};

} // end of namespace


int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_odometer");
  if (ros::names::remap("stereo") == "stereo") {
    ROS_WARN("'stereo' has not been remapped! Example command-line usage:\n"
             "\t$ rosrun viso2_ros stereo_odometer stereo:=narrow_stereo image:=image_rect");
  }
  if (ros::names::remap("image").find("rect") == std::string::npos) {
    ROS_WARN("stereo_odometer needs rectified input images. The used image "
             "topic is '%s'. Are you sure the images are rectified?",
             ros::names::remap("image").c_str());
  }

  std::string transport = argc > 1 ? argv[1] : "raw";
  viso2_ros::StereoOdometer odometer(transport);
  ros::spin();
  return 0;
}
