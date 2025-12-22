#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "bspline/non_uniform_bspline.h"
#include "mpc_tracking/Bspline.h"
#include "std_msgs/Empty.h"
#include "mpc_tracking/mpc.h"

#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

// #define BACKWARD_HAS_DW 1
// #include "backward.hpp"
// namespace backward{
//     backward::SignalHandling sh;
// }

using fast_planner::NonUniformBspline;

ros::Publisher cmd_vel_pub, motion_path_pub, predict_path_pub;
nav_msgs::Path predict_path, motion_path;
nav_msgs::Odometry odom;

// === 性能指标发布器（用于rosbag记录） ===
ros::Publisher tracking_error_pub;      // 跟踪误差 [lateral, heading, euclidean]
ros::Publisher solve_time_pub;          // MPC求解时间 (ms)
ros::Publisher smoothness_pub;          // 平滑度 [accel_x, accel_y, jerk]
ros::Publisher velocity_pub;            // 实际速度 [vx, vy, omega]


bool receive_traj = false;
vector<NonUniformBspline> traj;
double traj_duration;
ros::Time start_time;

double last_yaw;
double time_forward;

vector<Eigen::Vector3d> traj_cmd, traj_real;

ros::Timer control_cmd_pub, path_pub;

// === 性能指标变量 ===
geometry_msgs::Twist last_cmd;          // 上一次控制指令
ros::Time last_cmd_time;                // 上一次控制时间

const int N = 60;
const double dt = 0.1;

Eigen::Vector3d current_state;

unique_ptr<Mpc> mpc_ptr;

void bsplineCallback(mpc_tracking::BsplineConstPtr msg) {
  // parse pos traj

  Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);

  Eigen::VectorXd knots(msg->knots.size());
  for (int i = 0; i < msg->knots.size(); ++i) {
    knots(i) = msg->knots[i];
  }

  for (int i = 0; i < msg->pos_pts.size(); ++i) {
    pos_pts(i, 0) = msg->pos_pts[i].x;
    pos_pts(i, 1) = msg->pos_pts[i].y;
    pos_pts(i, 2) = msg->pos_pts[i].z;
  }

  NonUniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // parse yaw traj

  Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  for (int i = 0; i < msg->yaw_pts.size(); ++i) {
    yaw_pts(i, 0) = msg->yaw_pts[i];
  }

  NonUniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  start_time = msg->start_time;

  traj.clear();
  traj.push_back(pos_traj);
  traj.push_back(traj[0].getDerivative());
  traj.push_back(traj[1].getDerivative());
  traj.push_back(yaw_traj);
  traj.push_back(yaw_traj.getDerivative());

  traj_duration = traj[0].getTimeSum();

  receive_traj = true;
}

void replanCallback(std_msgs::Empty msg) {
  /* reset duration */
  const double time_out = 0.01;
  ros::Time time_now = ros::Time::now();
  double t_stop = (time_now - start_time).toSec() + time_out;
  traj_duration = min(t_stop, traj_duration);
}

void odomCallback(const nav_msgs::Odometry &msg) {
    odom = msg;
    current_state(0) = msg.pose.pose.position.x;
    current_state(1) = msg.pose.pose.position.y;
    current_state(2) = tf2::getYaw(msg.pose.pose.orientation);

    //double yaw1 = tf2::getYaw(msg.pose.pose.orientation);
    // Eigen::Quaterniond quaternion;
    // quaternion.x() = msg.pose.pose.orientation.x;
    // quaternion.y() = msg.pose.pose.orientation.y;
    // quaternion.z() = msg.pose.pose.orientation.z;
    // quaternion.w() = msg.pose.pose.orientation.w;


    // Eigen::Matrix3d rotationMatrix = quaternion.toRotationMatrix();
    // double yaw2 = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));
    cout << "x:" << current_state(0) << " " << "y:" << current_state(1) << endl;
    cout << "yaw1:" << current_state(2) << endl;
    //cout << "yaw2:" << yaw2 << endl;

}

void publish_control_cmd(const ros::TimerEvent &e) {
    if (!receive_traj) return;

    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - start_time).toSec();
    
    Eigen::Vector3d pos, vel, acc, pos_f;
    double yaw, yawdot;

    Eigen::MatrixXd desired_state = Eigen::MatrixXd::Zero(N, 3);

    if (t_cur + (N-1) * dt <= traj_duration && t_cur > 0) {
      for (int i = 0; i < N; ++i) {
        pos = traj[0].evaluateDeBoorT(t_cur + i * dt);
        vel = traj[1].evaluateDeBoorT(t_cur + i * dt);
        acc = traj[2].evaluateDeBoorT(t_cur + i * dt);
        yaw = traj[3].evaluateDeBoorT(t_cur + i * dt)[0];
        yawdot = traj[4].evaluateDeBoorT(t_cur + i * dt)[0];

        desired_state(i, 0) = pos[0];
        desired_state(i, 1) = pos[1];
        desired_state(i, 2) = yaw;
      }

    } else if (t_cur + (N-1) * dt > traj_duration && t_cur < traj_duration) {
        int more_num = (t_cur + (N-1) * dt - traj_duration) / dt;
        for (int i = 0; i < N - more_num; ++i) {
          pos = traj[0].evaluateDeBoorT(t_cur + i * dt);
          vel = traj[1].evaluateDeBoorT(t_cur + i * dt);
          acc = traj[2].evaluateDeBoorT(t_cur + i * dt);
          yaw = traj[3].evaluateDeBoorT(t_cur + i * dt)[0];
          yawdot = traj[4].evaluateDeBoorT(t_cur + i * dt)[0];

          desired_state(i, 0) = pos(0);
          desired_state(i, 1) = pos(1);
          desired_state(i, 2) = yaw;          
        }
        for (int i = N - more_num; i < N; ++i) {
          pos = traj[0].evaluateDeBoorT(traj_duration);
          vel.setZero();
          acc.setZero();
          yaw = traj[3].evaluateDeBoorT(traj_duration)[0];
          yawdot = traj[4].evaluateDeBoorT(traj_duration)[0];

          desired_state(i, 0) = pos(0);
          desired_state(i, 1) = pos(1);
          desired_state(i, 2) = yaw;
        }
    } else if (t_cur >= traj_duration)  {
      pos = traj[0].evaluateDeBoorT(traj_duration);
      vel.setZero();
      acc.setZero();
      yaw = traj[3].evaluateDeBoorT(traj_duration)[0];
      yawdot = traj[4].evaluateDeBoorT(traj_duration)[0];
      for (int i = 0; i < N; ++i) {
          desired_state(i, 0) = pos(0);
          desired_state(i, 1) = pos(1);
          desired_state(i, 2) = yaw;
      }
    } else {
      cout << "[Traj server]: invalid time." << endl;
  }

    // === 计算MPC求解时间 ===
    ros::Time solve_start = ros::Time::now();
    auto result = mpc_ptr->solve(current_state, desired_state);
    double solve_time_ms = (ros::Time::now() - solve_start).toSec() * 1000.0;
    
    geometry_msgs::Twist cmd;
    cmd.linear.x = result[0];   // u: forward velocity
    cmd.linear.y = result[1];   // v: lateral velocity
    //cmd.angular.z = result[2];  // r: angular velocity
    cmd_vel_pub.publish(cmd);
    //cout << "u:" << result[0] << " " << "r:" << result[1] << endl;
    
    // === 发布性能指标（用于rosbag记录） ===
    if (t_cur > 0 && t_cur <= traj_duration) {
        // 1. 跟踪误差 [lateral_error, heading_error, euclidean_error]
        double x_ref = desired_state(0, 0);
        double y_ref = desired_state(0, 1);
        double yaw_ref = desired_state(0, 2);
        
        double e_x = current_state(0) - x_ref;
        double e_y = current_state(1) - y_ref;
        double lateral_error = -e_x * sin(yaw_ref) + e_y * cos(yaw_ref);
        double heading_error = atan2(sin(current_state(2) - yaw_ref), cos(current_state(2) - yaw_ref));
        double euclidean_error = sqrt(e_x * e_x + e_y * e_y);
        
        std_msgs::Float64MultiArray error_msg;
        error_msg.data = {lateral_error, heading_error * 180.0 / M_PI, euclidean_error};
        tracking_error_pub.publish(error_msg);
        
        // 2. MPC求解时间
        std_msgs::Float64 solve_time_msg;
        solve_time_msg.data = solve_time_ms;
        solve_time_pub.publish(solve_time_msg);
        
        // 3. 平滑度指标 [accel_x, accel_y, jerk]
        if (last_cmd_time.toSec() > 0) {
            double dt_smooth = (time_now - last_cmd_time).toSec();
            if (dt_smooth > 0.001) {  // 避免除零
                double accel_x = (cmd.linear.x - last_cmd.linear.x) / dt_smooth;
                double accel_y = (cmd.linear.y - last_cmd.linear.y) / dt_smooth;
                double jerk = sqrt(accel_x * accel_x + accel_y * accel_y) / dt_smooth;
                
                std_msgs::Float64MultiArray smooth_msg;
                smooth_msg.data = {accel_x, accel_y, jerk};
                smoothness_pub.publish(smooth_msg);
            }
        }
        
        // 4. 实际速度 [vx, vy, omega]
        std_msgs::Float64MultiArray vel_msg;
        vel_msg.data = {odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z};
        velocity_pub.publish(vel_msg);
        
        // 更新上一次指令
        last_cmd = cmd;
        last_cmd_time = time_now;
    }

    predict_path.header.frame_id = "odom";
    predict_path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose_msg;
    geometry_msgs::Point pt;
    for (int i = 2; i < result.size(); i += 2) {
        pose_msg.pose.position.x = result[i];
        pose_msg.pose.position.y = result[i + 1];
        predict_path.poses.push_back(pose_msg);
    }
    predict_path_pub.publish(predict_path);
    
    predict_path.poses.clear();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_tracking_node");
    ros::NodeHandle nh("~");  // 使用私有命名空间

    // === 参数化配置（适配 hf_platform） ===
    std::string cmd_vel_topic, odom_topic;
    nh.param<std::string>("cmd_vel_topic", cmd_vel_topic, "/hf_platform/twist_mux/cmd_vel");
    nh.param<std::string>("odom_topic", odom_topic, "/odom");
    
    // 发布器
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
    predict_path_pub = nh.advertise<nav_msgs::Path>("/mpc_predict_path", 1);
    motion_path_pub = nh.advertise<nav_msgs::Path>("/mpc_motion_path", 1);
    
    // === 性能指标发布器（用于rosbag记录） ===
    tracking_error_pub = nh.advertise<std_msgs::Float64MultiArray>("/mpc_tracking/tracking_error", 10);
    solve_time_pub = nh.advertise<std_msgs::Float64>("/mpc_tracking/solve_time", 10);
    smoothness_pub = nh.advertise<std_msgs::Float64MultiArray>("/mpc_tracking/smoothness", 10);
    velocity_pub = nh.advertise<std_msgs::Float64MultiArray>("/mpc_tracking/velocity", 10);
   
    // 订阅器
    ros::Subscriber odom_sub = nh.subscribe(odom_topic, 1, &odomCallback);
    ros::Subscriber bspline_sub = nh.subscribe("/planning/bspline", 10, bsplineCallback);
    ros::Subscriber replan_sub = nh.subscribe("/planning/replan", 10, replanCallback);

    // 初始化 MPC 求解器
    mpc_ptr.reset(new Mpc());
    
    control_cmd_pub = nh.createTimer(ros::Duration(0.1), publish_control_cmd);
    
    ROS_INFO("=================================================");
    ROS_INFO("[MPC Tracking] Node started with configuration:");
    ROS_INFO("  cmd_vel_topic: %s", cmd_vel_topic.c_str());
    ROS_INFO("  odom_topic: %s", odom_topic.c_str());
    ROS_INFO("=================================================");

    ros::spin();
    return 0;

}


// rosbag record -O performance_test.bag \
//   /mpc_tracking/tracking_error \
//   /mpc_tracking/solve_time \
//   /mpc_tracking/smoothness \
//   /mpc_tracking/velocity \
//   /odom \
//   /cmd_vel \
//   /planning/bspline


