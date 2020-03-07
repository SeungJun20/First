#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>
#include <iostream>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <functional>
#include <ignition/math/Vector3.hh>
#include "gazebo_msgs/LinkStates.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"






namespace gazebo                          
{
    
    
    

    
    
class PIDJoints : public ModelPlugin             
  {
    physics::LinkPtr body;    
    physics::LinkPtr right_wheel;    
    physics::LinkPtr left_wheel;    
    physics::JointPtr right_wheel_joint;                     
    physics::JointPtr left_wheel_joint;                     
    physics::ModelPtr model;
    
    common::Time last_update_time;
    event::ConnectionPtr update_connection;
    double dt;
    double time=0;
    
    sensors::SensorPtr Sensor;
    sensors::ImuSensorPtr IMU;
    double IMU_Update;
    double angular_velocity_x;
    double angular_velocity_y;
    double angular_velocity_z;
    double linear_acc_x = 0;
    double linear_acc_y = 0;
    double linear_acc_z = 0;

    double PI=3.141592;
    double angle_x = 0;
    double angle_y = 0;
    double angle_z = 0;
    double R=0;
    double angle_y_from_acc=0;
    double angle_y_from_acc_m=0;
    double right_motor_Torque, left_motor_Torque;
    double right_wheel_Torque, left_wheel_Torque;
    double right_wheel_angle, left_wheel_angle;
    double pre_right_wheel_angle = 0, pre_left_wheel_angle = 0;
    double right_wheel_angular_velocity, left_wheel_angular_velocity;
    double right_wheel_angular_acc, left_wheel_angular_acc;
    double theta_Kp = -38.2797;
    double theta_Kd = -13.0518;
    double Kol = 2.0;
    double phi_Kp = -1/2;
    double phi_Kd = -2.7132/2;
      
    double cnt=0;
    
    double world_angle_y=0;
    math::Pose world_pose;
    math::Vector3 world_vector3;
    math::Quaternion world_Quaternion;

    double angle_y_m=0;   
    double angle_y_comp=0;
    double comp_gain=0.999;
    
    double desired_angle = 0;
    double total_angle = 0;
    
    
    ros::Subscriber sub;
    
    ros::NodeHandle n;
    
    
    
    
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      
      this->model = _model;
      this->body = this->model->GetLink("body");
      this->right_wheel_joint = this->model->GetJoint("R_Wheel_joint");
      this->left_wheel_joint = this->model->GetJoint("L_Wheel_joint");
      
      this->last_update_time = this->model->GetWorld()->GetSimTime();
      this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&PIDJoints::UpdatePID, this));

      this->Sensor = sensors::get_sensor("IMU");
      this->IMU = std::dynamic_pointer_cast<sensors::ImuSensor>(Sensor);
     
     sub = n.subscribe("Angular",1000 , &gazebo::PIDJoints::callback, this);
     
     }
  
void callback(const std_msgs::Float64Ptr &msg)
{
    desired_angle = msg -> data;
    
    
     
}

        
    
    
    
    
      void UpdatePID()
    {
        
common::Time current_time = this->model->GetWorld()->GetSimTime();
      dt = current_time.Double() - this->last_update_time.Double();
      time=time+dt;
      if(time==0)
      {   
          
          angle_x =0 ;
          angle_y =0 ;
          angle_z =0 ;  
          angular_velocity_x = 0;
          angular_velocity_y = 0;
          angular_velocity_z = 0;  
      }
      
      angular_velocity_x = this->IMU->AngularVelocity(false)[0];
      angular_velocity_y = this->IMU->AngularVelocity(false)[1];
      angular_velocity_z = this->IMU->AngularVelocity(false)[2];
      angle_x =  angle_x + angular_velocity_x*dt;
      angle_y =  angle_y + angular_velocity_y*dt;
      angle_z =  angle_z + angular_velocity_z*dt;
      
      linear_acc_x = this->IMU->LinearAcceleration(false)[0];
      linear_acc_y = this->IMU->LinearAcceleration(false)[1];
      linear_acc_z = this->IMU->LinearAcceleration(false)[2];
      R=sqrt(pow(linear_acc_x,2)+pow(linear_acc_y,2)+pow(linear_acc_z,2))/1000;
      angle_y_from_acc=atan((sqrt(pow(linear_acc_x,2)+pow(linear_acc_y,2)))/R);
      
      if(math::isnan(angle_y_from_acc))
      {
          angle_y_from_acc=angle_y_from_acc_m;
      }
          angle_y_from_acc_m=angle_y_comp;
              
      angle_y_comp = (comp_gain*(angle_y_comp+linear_acc_y*dt)+(1-comp_gain)*angle_y_from_acc);
     
      if(math::isnan(angle_y_comp))
      {
          angle_y_comp=angle_y_m;
      }
        angle_y_m=angle_y_comp;
        
      world_Quaternion = this->body->GetWorldPose().rot;
      world_angle_y=world_Quaternion.GetAsEuler().y;

      
      
     


      
      right_wheel_angle = this->right_wheel_joint->GetAngle(1).Degree()+(angle_y)*180./PI;
      right_wheel_angular_velocity = (right_wheel_angle-pre_right_wheel_angle)/dt;
      pre_right_wheel_angle = right_wheel_angle;
     
      left_wheel_angle = this->left_wheel_joint->GetAngle(1).Degree()+(angle_y)*180./PI;
      left_wheel_angular_velocity = (left_wheel_angle-pre_left_wheel_angle)/dt;
      pre_left_wheel_angle = left_wheel_angle;
     
      total_angle = this->right_wheel_joint->GetAngle(1).Radian();
      
      if(total_angle >= 26){
          desired_angle = 0;
      }
      if(total_angle < 26){
          desired_angle = 0.05;
      }
      
      
      right_motor_Torque = (theta_Kp*(desired_angle - angle_y)-theta_Kd*(angular_velocity_y)-phi_Kp*(right_wheel_angle*PI/180.)-phi_Kd*(PI/180.*right_wheel_angular_velocity));
      
      //right_motor_Torque = -(theta_Kp*(angle_y)+theta_Kd*(angular_velocity_y)+phi_Kp*(right_wheel_angle*PI/180.)+phi_Kd*(PI/180.*right_wheel_angular_velocity));
      
      //right_motor_Torque = -(theta_Kp*(angle_y)+theta_Kd*(angular_velocity_y)+phi_Kp*(right_wheel_angle*PI/180.)+phi_Kd*(PI/180.*right_wheel_angular_velocity)+Kol*(desired_angle));
      
      left_motor_Torque = (theta_Kp*(desired_angle - angle_y)-theta_Kd*(angular_velocity_y)-phi_Kp*(left_wheel_angle*PI/180.)-phi_Kd*(PI/180.*left_wheel_angular_velocity));
      
      //left_motor_Torque = -(theta_Kp*(angle_y)+theta_Kd*(angular_velocity_y)+phi_Kp*(left_wheel_angle*PI/180.)+phi_Kd*(PI/180.*left_wheel_angular_velocity)+Kol*(desired_angle)); 
      
      right_wheel_Torque = right_motor_Torque;
      left_wheel_Torque = left_motor_Torque+(1.0*(right_wheel_angle-left_wheel_angle)+0.05*(right_wheel_angular_velocity-left_wheel_angular_velocity));
      
      
      this->right_wheel_joint->SetForce(1,right_wheel_Torque);   //setForce(axis,Force value)
      this->left_wheel_joint->SetForce(1, left_wheel_Torque);
      this->last_update_time = current_time;
    
     // std::cout<<angle_y<<std::endl;
      std::cout<<total_angle<<std::endl;
    
      }
  };
    GZ_REGISTER_MODEL_PLUGIN(PIDJoints);
    
    
}


