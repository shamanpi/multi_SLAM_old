#include "hw2.h"
//#include "visualization/vis.h"
//#include "visualization/opengl/glvis.h"

#include <random>
#include <iostream>
#include <ros/ros.h>
#include <feature_extraction/corners_and_lines.h>
#include <geometry_msgs/Twist.h>

#ifndef M_PI
#define M_PI 3.141592653589793
#endif
#ifndef M_PI_2
#define M_PI_2 1.5707963267948966
#endif

// Landmark and movement definitions, for use with the callback functions
std::vector<Eigen::VectorXd> landmarks_0(50, Eigen::Vector2d::Zero());
std::vector<Eigen::VectorXd> landmarks_1(50, Eigen::Vector2d::Zero());
Eigen::VectorXd u(4);


void TestRelPosSLAM(ros::Publisher& slam_pub, feature_extraction::corners_and_lines& slam_msg) {
  //Eigen::VectorXd x_TRUE2 = Eigen::Vector3d(-2.0, 0.0, M_PI_2);
  double std_x = 0.05;  // initial covariance
  
  Eigen::MatrixXd Sigma_t = std_x * std_x * Eigen::MatrixXd::Identity(6,6);
  Eigen::VectorXd x_t <<-12.0, 11.0, M_PI_2,-12.0, 7.0, -M_PI_2;

  // Noise Covariance:
  double std_n = 0.03;
  Eigen::MatrixXd Sigma_n = std_n * std_n * Eigen::MatrixXd::Identity(4,4);
  double std_m = 0.03;
  Eigen::Matrix2d Sigma_m = std_m * std_m * Eigen::MatrixXd::Identity(2,2);

  double dt = 0.1;
  int i = 0;
  ros::Rate r(100);
  while(ros::ok()) {    
    // Run an EKFSLAMPropagation step
    Eigen::VectorXd x_new;
    Eigen::MatrixXd Sigma_new;
    EKFSLAMPropagate(x_t, Sigma_t, u, Sigma_n, dt, x_new, Sigma_new);
    x_t = x_new;
    Sigma_t = Sigma_new;

    // Run an EKFSLAMUpdate step
    int noofcorners = landmarks_0.size();
    if (noofcorners > 0) {
      Eigen::VectorXd x_new;
      Eigen::MatrixXd Sigma_new;
      std::vector<Eigen::MatrixXd> close_Sigma_m;  // covariance in landmark pose msmt
      //landmark noise
      for(size_t w = 0; w < noofcorners; ++w){
	close_Sigma_m.push_back(Sigma_m);
      }
      
      int per = 0;
      EKFSLAMRelPosUpdate(x_t, Sigma_t, landmarks_0, close_Sigma_m, x_new, Sigma_new,per);
      x_t = x_new;
      Sigma_t = Sigma_new;
    }//ifend

    //Run an EKFSLAMUpdate step
    noofcorners = landmarks_1.size();
    if (noofcorners > 0) {//if corners from robot 1 is greater than zero
      Eigen::VectorXd x_new;
      Eigen::MatrixXd Sigma_new;
      std::vector<Eigen::MatrixXd> close_Sigma_m;
      //LANDMARK NOISE
      for(size_t w = 0; w < noofcorners; ++w){
        close_Sigma_m.push_back(Sigma_m);
      }
      
      int per = 1;
      //EKFSLAMRelPosUpdate
      EKFSLAMRelPosUpdate(x_t, Sigma_t, landmarks_1, close_Sigma_m, x_new, Sigma_new,per);
      x_t = x_new;
      Sigma_t = Sigma_new;
    }


    std::vector<float> corners(x_t.size(), 0);
    std::vector<float> covs(Sigma_t.rows(), 0);
    int i = 0;
    for (int it = 0; it != x_t.size(); ++it, ++i)
      corners[i] = x_t[it];
    i = 0;
    Eigen::VectorXd diag = Sigma_t.diagonal();
    for (int it = 0; it != diag.size(); ++it, ++i)
      covs[i] = diag[it];
    
      
    slam_msg.corners = corners; // needs to go into a float32[]
    slam_msg.endPoints = covs; // needs to go into a float32[]
    slam_pub.publish(slam_msg); // publish new x_hat value with variances


    i += 1;
    r.sleep(); // Aim for about 10 Hz processing
  }
}


void landmarks_0_callback(const feature_extraction::corners_and_lines::ConstPtr& msg){
  std::vector<float> tb3_0_msmts = msg->corners;
  std::vector<Eigen::VectorXd> landmarks_0(tb3_0_msmts.size() / 2, Eigen::Vector2d::Zero());
  std::vector<Eigen::VectorXd>::iterator l_it = landmarks_0.begin();
  for (std::vector<float>::iterator it = tb3_0_msmts.begin() ; it != tb3_0_msmts.end(); ++it, ++l_it){
    Eigen::VectorXd dummy = Eigen::Vector2d::Zero();
    dummy[0] = *it;
    it++;
    dummy[1] = *it;
    *l_it = dummy;
  }
}

void landmarks_1_callback(const feature_extraction::corners_and_lines::ConstPtr& msg){
  std::vector<float> tb3_1_msmts = msg->corners;
  std::vector<Eigen::VectorXd> landmarks_1(tb3_1_msmts.size() / 2, Eigen::Vector2d::Zero());
  std::vector<Eigen::VectorXd>::iterator l_it = landmarks_1.begin();
  for (std::vector<float>::iterator it = tb3_1_msmts.begin() ; it != tb3_1_msmts.end(); ++it, ++l_it){
    Eigen::VectorXd dummy = Eigen::Vector2d::Zero();
    dummy[0] = *it;
    it++;
    dummy[1] = *it;
    *l_it = dummy;
  }
}

void vel_0_callback(const geometry_msgs::Twist::ConstPtr& msg){
  u[0] = msg->linear.x;
  u[1] = msg->angular.z;
}

void vel_1_callback(const geometry_msgs::Twist::ConstPtr& msg){
  u[2] = msg->linear.x;
  u[3] = msg->angular.z;
}

int main(int argc, char** argv) {

  {
    // specific node name and handle
    ros::init(argc, argv,"slam");
    ros::NodeHandle nh;
    ros::Publisher slam_pub = nh.advertise<feature_extraction::corners_and_lines>("slam_results", 1000);
    feature_extraction::corners_and_lines slam_msg;
    
    // subscribe to the feature and commanded velocity topics
    ros::Subscriber tb3_0_featureSub = nh.subscribe("tb3_0/features", 100, landmarks_0_callback);
    ros::Subscriber tb3_1_featureSub = nh.subscribe("tb3_1/features", 100,landmarks_1_callback);
    ros::Subscriber tb3_0_velSub = nh.subscribe("/tb3_0/cmd_vel", 100, vel_0_callback);
    ros::Subscriber tb3_1_velSub = nh.subscribe("/tb3_1/cmd_vel", 100, vel_1_callback);


    //std::cout << tb3_0_msmts << '\n';
    // Now generate a new scene for SLAM and try to Update estimates through it
    TestRelPosSLAM(slam_pub, slam_msg);

    // run until stopped
    // ros::spin();
  }
}
