#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <stdint.h>
#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

#define MOTOR_COEFFICIENT 19.4679 // Thrust N/unit of total motor signal


struct UAMState{
	Eigen::Vector3d vel_enu;
	Eigen::Vector3d vel_ned;
	Eigen::Vector3d rpy_enu;
	Eigen::Vector3d rpy_ned;
	Eigen::Vector4d actuators;
	double a_tot;
	uint64_t base_stamp;
	Eigen::Vector3d force_est;
	Eigen::Vector3d vel_est;
	Eigen::Vector3d accel;
	Eigen::Vector3d est_err_prev;

	UAMState() : rpy_enu(Eigen::Vector3d::Zero()), rpy_ned(Eigen::Vector3d::Zero()),
	              vel_enu(Eigen::Vector3d::Zero()), vel_ned(Eigen::Vector3d::Zero()), actuators(Eigen::Vector4d::Zero()), base_stamp(0),
	              force_est(Eigen::Vector3d::Zero()), vel_est(Eigen::Vector3d::Zero()), a_tot(0), accel(Eigen::Vector3d::Zero()),
	              est_err_prev(Eigen::Vector3d::Zero()) { }
};

class UAMEstimator : public rclcpp::Node
{
public:
	UAMEstimator() : Node("ext_force_estimator")
	{
    HAS_VICON = false;
    HAS_ACT = false;
    start_working = false;
          
    int8_t qos_depth = 10;
  	const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).best_effort().durability_volatile();
	  vicon_sub = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", QOS_RKL10V, std::bind(&UAMEstimator::vicon_callback, this, std::placeholders::_1));
    actuators_sub = this->create_subscription<px4_msgs::msg::ActuatorMotors>("/fmu/out/actuator_motors", QOS_RKL10V, std::bind(&UAMEstimator::actuators_callback, this, std::placeholders::_1));
	  force_est_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("/uam/force_estimate", 10);          
          	  
    rclcpp::Time t_ = this->get_clock()->now();
	  start_time = t_.seconds();   
	  last_time_act = start_time;
	  last_time_odom = start_time;
	  timer_ = this->create_wall_timer(50ms, std::bind(&UAMEstimator::timer_callback, this));
  }

        void timer_callback();
	void vicon_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
	void actuators_callback(const px4_msgs::msg::ActuatorMotors::SharedPtr msg);	
	Eigen::Matrix3d Rx(double x);
	Eigen::Matrix3d Ry(double x);
	Eigen::Matrix3d Rz(double x);	
	
private:
	rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vicon_sub;
  rclcpp::Subscription<px4_msgs::msg::ActuatorMotors>::SharedPtr actuators_sub;   
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr force_est_publisher_;

  std::atomic<uint64_t> timestamp_odom;   //!< common synced timestamped
  std::atomic<uint64_t> timestamp_act;	
  double start_time;
  double last_time_act;
  double last_time_odom;        
  UAMState UAM_STATE;
  bool HAS_VICON;
  bool HAS_ACT;
  bool start_working;
        
};
//==================================================//
void UAMEstimator::timer_callback()
{
  // If not started yet, don't do anything
  if(!start_working){
    return;
  }

  // NED to ENU rotation matrix
  Eigen::Matrix3d R = Rz(M_PI/2.0) * Rx(-M_PI);
  Eigen::Vector3d force_w = R * UAM_STATE.force_est;
  geometry_msgs::msg::Vector3 force_msg;
  force_msg.x = force_w(0);
  force_msg.y = force_w(1);
  force_msg.z = force_w(2);
  force_est_publisher_->publish(force_msg);
}

// Calculate expected velocities based on actuator action
void UAMEstimator::actuators_callback(const px4_msgs::msg::ActuatorMotors::SharedPtr msg)
{
  // Don't do anything if the message is incorrect
  if(msg->control.size() < 4){
   return;
  }

  // If HAS_ACT is false (i.e. when just initialized)
  if(!HAS_ACT){
    HAS_ACT = true;
    UAM_STATE.actuators(0) = msg->control[0];
    UAM_STATE.actuators(1) = msg->control[1];
    UAM_STATE.actuators(2) = msg->control[2];
    UAM_STATE.actuators(3) = msg->control[3];
    UAM_STATE.a_tot = UAM_STATE.actuators(0)+UAM_STATE.actuators(1)+UAM_STATE.actuators(2)+UAM_STATE.actuators(3);
    return;
  }

  // If there is not mocap, don't do anything
  if(!HAS_VICON){
    return;
  }


  timestamp_act = msg->timestamp;
  rclcpp::Time t = this->get_clock()->now();

  // If mocap is enabled and UAM_STATE.actuators is populated with the first message
  if(HAS_VICON && HAS_ACT && !start_working){
    start_working = true;
    start_time = t.seconds();
    last_time_act = start_time;

    // Take oldest timestamp
    if(timestamp_act <= timestamp_odom){
      UAM_STATE.base_stamp = timestamp_act;
    }else{
      UAM_STATE.base_stamp = timestamp_odom;
    }
    return; //return so that only a valid dt will be used
  }
  // Continue below if start_working set to true i.e. if above block has been executed once

  double dt = t.seconds() - last_time_act; // dt is time between actuator messages
  if(dt < 5e-3){ // In case of small timestep, don't do anything (I assume to avoid div0 errors)
    return;
  }
  last_time_act = t.seconds();
  
  // Update UAM_STATE
  UAM_STATE.actuators(0) = msg->control[0];
  UAM_STATE.actuators(1) = msg->control[1];
  UAM_STATE.actuators(2) = msg->control[2];
  UAM_STATE.actuators(3) = msg->control[3];
  double a_tot = UAM_STATE.actuators(0)+UAM_STATE.actuators(1)+UAM_STATE.actuators(2)+UAM_STATE.actuators(3);
  UAM_STATE.a_tot = 0.7*UAM_STATE.a_tot + 0.3*a_tot; // EWMA filter for smoothness

  double T = MOTOR_COEFFICIENT*UAM_STATE.a_tot; // Put here some drone constant (T is thrust accleration? and a_tot the cumulative normalized motor input)
  Eigen::Matrix3d R = Rz(UAM_STATE.rpy_ned(2))*Ry(UAM_STATE.rpy_ned(1))*Rx(UAM_STATE.rpy_ned(0));
  Eigen::Matrix3d D_ = 0.2*Eigen::Matrix3d::Identity();
  // Rotate thrust to world frame, subtract gravity... The rest I'm not sure about
  Eigen::Vector3d accel = R*Eigen::Vector3d(0,0,-T) - Eigen::Vector3d(0,0,-9.81) + (1.0/2.46)*UAM_STATE.force_est - (1.0/2.46)*D_*UAM_STATE.vel_ned ;
  UAM_STATE.accel = 0.5*UAM_STATE.accel + 0.5*accel; // Another EWMA filter
  UAM_STATE.vel_est = UAM_STATE.vel_est + dt * UAM_STATE.accel;
}

void UAMEstimator::vicon_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  tf2::Quaternion q_local(msg->q.at(1), msg->q.at(2), msg->q.at(3), msg->q.at(0));
  tf2::Matrix3x3 R(q_local); //drone to drone's idea of mocap ??
  double roll, pitch, yaw;
  R.getRPY(roll,pitch,yaw);
  UAM_STATE.rpy_enu = Eigen::Vector3d(roll, pitch, yaw);
  UAM_STATE.vel_enu = Eigen::Vector3d(msg->velocity.at(0), -msg->velocity.at(1), -msg->velocity.at(2));
  rclcpp::Time t = this->get_clock()->now();  
  
  // If this is the first mocap message, set HAS_VICON to true
  if(!(this->HAS_VICON)){
    R.getRPY(roll,pitch,yaw);
    UAM_STATE.rpy_ned = Eigen::Vector3d(roll, pitch, yaw);
    UAM_STATE.vel_ned = Eigen::Vector3d(msg->velocity.at(0), msg->velocity.at(1), msg->velocity.at(2));
    this->HAS_VICON = true;
    last_time_odom = t.seconds();
    return;
  }

  // EWMA on RPY ned and velocity from Mocap
  R.getRPY(roll,pitch,yaw);
  UAM_STATE.rpy_ned = 0.7*UAM_STATE.rpy_ned + 0.3*Eigen::Vector3d(roll, pitch, yaw);
  UAM_STATE.vel_ned = 0.95*UAM_STATE.vel_ned + 0.05*Eigen::Vector3d(msg->velocity.at(0), msg->velocity.at(1), msg->velocity.at(2));

  timestamp_odom = msg->timestamp;
  if(start_working){
    double dt = t.seconds() - last_time_odom;
    if(dt < 5e-3){
      return;
    }
    last_time_odom = t.seconds();
    Eigen::Vector3d est_error = UAM_STATE.vel_ned - UAM_STATE.vel_est; // Difference between Mocap velocity and integrated velocity
    Eigen::Vector3d est_err_dot = (est_error - UAM_STATE.est_err_prev)/dt;
    UAM_STATE.est_err_prev = est_error;
    Eigen::Vector3d f_est_dot = 15.0*est_error + 27.0*est_err_dot; // What are these numbers
    UAM_STATE.force_est = UAM_STATE.force_est + dt*f_est_dot;
    //here, compute velocity error, compute f_dot with local dt, update, force_est
  }
}

Eigen::Matrix3d UAMEstimator::Rx(double x){
	Eigen::Matrix3d result;
	result << 1.0,0.0,0.0,0.0,cos(x),-sin(x),0.0,sin(x),cos(x) ;
	return result;
}

Eigen::Matrix3d UAMEstimator::Ry(double x){
	Eigen::Matrix3d result;
	result << cos(x),0.0,sin(x),0.0,1.0,0.0,-sin(x),0.0,cos(x) ; (1.0/2.46)*UAM_STATE.force_est;
	return result;
}

Eigen::Matrix3d UAMEstimator::Rz(double x){
	Eigen::Matrix3d result;
	result << cos(x),-sin(x),0.0,sin(x),cos(x),0.0,0.0,0.0,1.0 ;
	return result;
}


int main(int argc, char *argv[])
{
	std::cout << "Starting force estimator node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<UAMEstimator>());

	rclcpp::shutdown();
	return 0;
}

