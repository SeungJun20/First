///Just LQR SEGWAY//////


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
#include <iostream>





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
    
    double m1 = 24.0;
    double m2 = 4.0;
    double m3 = 2.0;
    double m4 = 12.3;
    double Mb = 37.65;
    
    double L1 = -0.085;
    double L2 = 0.034;
    double L3 = 0.5;
    double L4 = 0.65;
    double Lcg = 0.14;
    
    
    double d = 0.55;
    
    double I1 = 0.2105;
    double I2 = 0.023;
    double I3 = 0.167;
    double I4 = 0.033;
    double Icg = 4.64;
    
    
    double Mw = 0.85;
    double Rw = 0.1925;
    double Jw = 0.02;
    double g =9.8;
    double np = 13.715;
    double Jm = 1.412*pow(10,-4);
    double Bm = 3.78*pow(10,-4);
    double Kt = 3.756*pow(10,-2);
    double Ke = 3.756*pow(10,-2);
    double Rm = 0.0788;
    double F_d = 0;
    double Tw = 0;
    
    
    double left_motor_Torque, right_motor_Torque;
    
    
    double angle_x = 0;
    double angle_y = 0;
    double angle_z = 0;
    
    double angular_velocity_x;
    double angular_velocity_y;
    double angular_velocity_z;
    

    double PI=3.141592;
    
    double R=0;
    double angle_y_from_acc=0;
    double angle_y_from_acc_m=0;
    
    double right_wheel_Torque, left_wheel_Torque;
    double right_wheel_angle, left_wheel_angle;
    double pre_phi;
    double cnt=0;
    
    
    
    double theta = 0;
    double D_theta = 0;
    double D2_theta = 0;
    
    double L_phi = 0;
    double R_phi = 0;
    double D_L_phi = 0;
    double D_R_phi = 0;
    double D2_L_phi = 0;
    double D2_R_phi = 0;
    double pre_R_phi = 0;
    double pre_L_phi = 0;
    
    double New_L_phi = 0;
    double New_R_phi = 0;

    
    
    
    
    double world_angle_y=0;
    math::Pose world_pose;
    math::Vector3 world_vector3;
    math::Quaternion world_Quaternion;

    double desired_angle = 0;
    double desired_meter = 0;
    double total_phi = 0;
    double desired_speed = 0;
    double pre_desired_meter = 0;
    
    double a1 = -29.3711;
    double a2 = -0.7259;
    double a3 = -13.1508;
    double a4 = -1.6026;
    double b1 = 96.8054/20;
    double b2 = 1.8763/20;
    double b3 = 33.9920/20;
    double b4 = 4.1425/20;
    
    double K1 = -55.6929;
    double K2 = -1.0000/2;
    double K3 = -18.1165;
    double K4 = -2.2078/2;
    double Kol = 0.0;
    double Kp = 2.0;
    double Kd = 0.02;
    double target_distance = 0;
    double actual_distance = 0;
    double delta_d = 0;
    double target_velocity =0;
    
    
    ros::Subscriber sub_1;
    ros::Subscriber sub_2;
    
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
     
    sub_1 = n.subscribe("Speed",1000 , &gazebo::PIDJoints::callback_1, this);
    sub_2 = n.subscribe("meter",1000 , &gazebo::PIDJoints::callback_2, this); 
     }
  
void callback_1(const std_msgs::Float64Ptr &msg)
{
  target_velocity = msg -> data;
   
             
}
  void callback_2(const std_msgs::Float64Ptr &msg)
{
    delta_d = msg -> data;
    target_distance = delta_d + target_distance;
     

}
        
      void UpdatePID()
    {
        
common::Time current_time = this->model->GetWorld()->GetSimTime();
      dt = current_time.Double() - this->last_update_time.Double();
      time=time+dt;
      if(time==0)
      {   
    theta = 0;
    D_theta = 0;
    D2_theta = 0;
    
    L_phi = 0;
    R_phi = 0;
    D_L_phi = 0;
    D_R_phi = 0;
    D2_L_phi = 0;
    D2_R_phi = 0;
    angle_x =0 ;
    angle_z =0 ;  
    angular_velocity_x = 0;
    angular_velocity_z = 0; 

          
      }
      
 
         total_phi = this->right_wheel_joint->GetAngle(1).Radian();
         actual_distance = total_phi*Rw;  
  
      
      
      
   
          
      angular_velocity_x = this->IMU->AngularVelocity(false)[0];
      D_theta = this->IMU->AngularVelocity(false)[1];
      angular_velocity_z = this->IMU->AngularVelocity(false)[2];
      
   
      
      angle_x =  angle_x + angular_velocity_x*dt;
      theta =  theta + D_theta*dt;
      angle_z =  angle_z + angular_velocity_z*dt;
      
      
      D2_theta = a1*theta +a2*L_phi +a3*D_theta +a4*D_L_phi;
      
      
      
      D2_L_phi = b1*theta + b2*L_phi +b3*D_theta +b4*D_L_phi;
      D2_R_phi = b1*theta + b2*R_phi +b3*D_theta +b4*D_R_phi;
      
      R_phi = this->right_wheel_joint->GetAngle(1).Radian()+(theta);
      D_R_phi = (R_phi-pre_R_phi)/dt;
      New_R_phi = New_R_phi + R_phi - pre_R_phi;
      pre_R_phi = R_phi;
     
      L_phi = this->left_wheel_joint->GetAngle(1).Radian()+(theta);
      D_L_phi = (L_phi-pre_L_phi)/dt;
      New_L_phi = New_L_phi + L_phi - pre_L_phi;
      pre_L_phi = L_phi;
      
      world_Quaternion = this->body->GetWorldPose().rot;
      world_angle_y=world_Quaternion.GetAsEuler().y;
      

          
         K1 = -55.6929;
         K2 = -1.0000;
         K3 = -18.1165;
         K4 = -2.2078;
         Kol = 2;
         
          
         std::cout<<target_distance<<std::endl;
      
      
      //right_motor_Torque = ((Icg+Mb*pow(Lcg,2))*D2_theta+Mb*Rw*Lcg*D2_R_phi-Mb*g*Lcg*(theta-desired_angle))/-2;
      //left_motor_Torque = ((Icg+Mb*pow(Lcg,2))*D2_theta+Mb*Rw*Lcg*D2_L_phi-Mb*g*Lcg*(theta-desired_angle))/-2;
      
         
      right_motor_Torque = -K1*(theta) -K2*(R_phi) -K3*(D_theta) -K4*(D_R_phi)-Kp*(target_distance-actual_distance)-Kd*(target_velocity-D_theta);
      left_motor_Torque = -K1*(theta) -K2*(L_phi) -K3*(D_theta) -K4*(D_L_phi) -Kp*(target_distance-actual_distance)-Kd*(target_velocity-D_theta);
      
      
      
      
      //right_motor_Torque = (Mb*Rw*Lcg*D2_theta + 2*Jw+(2*Mw+Mb)*pow(Rw,2)*D2_R_phi)/2;
      //left_motor_Torque = (Mb*Rw*Lcg*D2_theta + 2*Jw+(2*Mw+Mb)*pow(Rw,2)*D2_L_phi)/2;
      
      right_wheel_Torque = right_motor_Torque;
      left_wheel_Torque = left_motor_Torque + (4*(R_phi-L_phi)+3*(D_R_phi-D_L_phi));
      
    
      
  //  std::cout<<left_wheel_Torque<<std::endl;
   // std::cout<<right_wheel_Torque<<std::endl;
      this->right_wheel_joint->SetForce(1,right_wheel_Torque);   //setForce(axis,Force value)
      this->left_wheel_joint->SetForce(1, left_wheel_Torque);
      this->last_update_time = current_time;

      std::cout<<actual_distance<<std::endl;
   
    
      }
  };
    GZ_REGISTER_MODEL_PLUGIN(PIDJoints);
    
    
}



