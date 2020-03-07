#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>
#include <iostream>
#include <std_msgs/Float64.h>
#include <functional>
#include <ignition/math/Vector3.hh>

namespace gazebo                          
{
 
class PIDJoints : public ModelPlugin             
  {
    physics::LinkPtr body;                                   
    physics::JointPtr right_wheel_joint;                     
    physics::JointPtr left_wheel_joint;                     
    physics::ModelPtr model;
    
    common::Time last_update_time;
    event::ConnectionPtr update_connection;
    double dt;
    
    sensors::SensorPtr Sensor;
    sensors::ImuSensorPtr IMU;
    double IMU_Update;
    double angular_velocity_x;
    double angular_velocity_y;
    double angular_velocity_z;
    double linear_acc_x = 0;
    double linear_acc_y = 0;
    double linear_acc_z = 0;
    
    ros::NodeHandle n;
    ros::Publisher P_Times;
    ros::Publisher P_angular_velocity_x;
    ros::Publisher P_angular_velocity_y;
    ros::Publisher P_angular_velocity_z;
    std_msgs::Float64 m_Times;
    std_msgs::Float64 m_angular_velocity_x;
    std_msgs::Float64 m_angular_velocity_y;
    std_msgs::Float64 m_angular_velocity_z;

     
 public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {  
      this->model = _model;

   this->body = this->model->GetLink("Body");
      this->right_wheel_joint = this->model->GetJoint("R_Wheel_joint");
      this->left_wheel_joint = this->model->GetJoint("L_Wheel_joint");

      this->last_update_time = this->model->GetWorld()->GetSimTime();
      this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&PIDJoints::UpdatePID, this));

      this->Sensor = sensors::get_sensor("IMU");
      this->IMU = std::dynamic_pointer_cast<sensors::ImuSensor>(Sensor);
     
      P_Times = n.advertise<std_msgs::Float64>("times",1);
      P_angular_velocity_x = n.advertise<std_msgs::Float64>("angular_velocity_x",1);
      P_angular_velocity_y = n.advertise<std_msgs::Float64>("angular_velocity_y",1);  
      P_angular_velocity_z = n.advertise<std_msgs::Float64>("angular_velocity_z",1);
      ros::Rate loop_rate(1000);
  }

    void UpdatePID()
    {
      common::Time current_time = this->model->GetWorld()->GetSimTime();
      dt = current_time.Double() - this->last_update_time.Double();
      
      this->right_wheel_joint->SetForce(1, 10);   //setForce(axis,Force value)
      this->left_wheel_joint->SetForce(1, 10);
              
      this->last_update_time = current_time;
       
      angular_velocity_x = this->IMU->AngularVelocity(false)[0];
      angular_velocity_y = this->IMU->AngularVelocity(false)[1];
      angular_velocity_z = this->IMU->AngularVelocity(false)[2];
      linear_acc_x = this->IMU->LinearAcceleration(false)[0];
      linear_acc_y = this->IMU->LinearAcceleration(false)[1];
      linear_acc_z = this->IMU->LinearAcceleration(false)[2];
    
      m_Times.data = current_time.Double();
      m_angular_velocity_x.data = angular_velocity_x;
      m_angular_velocity_y.data = angular_velocity_y;
      m_angular_velocity_z.data = angular_velocity_z;
      P_Times.publish(m_Times);
      P_angular_velocity_x.publish(m_angular_velocity_x);
      P_angular_velocity_y.publish(m_angular_velocity_y);
      P_angular_velocity_z.publish(m_angular_velocity_z);
 
    }
  };
    GZ_REGISTER_MODEL_PLUGIN(PIDJoints);
}