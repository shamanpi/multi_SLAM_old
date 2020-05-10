#include "hw2.h"
//#include "visualization/vis.h"
//#include "visualization/opengl/glvis.h"

#include <random>
#include <iostream>
#include <ros/ros.h>
#include <feature_extraction/corners_and_lines.h>
#include <cstring>

#ifndef M_PI
#define M_PI 3.141592653589793
#endif
#ifndef M_PI_2
#define M_PI_2 1.5707963267948966
#endif




void TestRelPosSLAM() {
  std::random_device rd;
  std::mt19937 mt(rd());
  std::normal_distribution<double> dist(0.0, 1.0);
  std::uniform_real_distribution<double> lm_pos_dist(-3.0, 3.0);

  std::vector<Eigen::VectorXd> landmarks(40, Eigen::Vector2d::Zero());
  // Generate Point Landmarks:
  for (size_t i = 0; i < landmarks.size(); ++i) {
    landmarks[i][0] = lm_pos_dist(mt);
    landmarks[i][1] = lm_pos_dist(mt);
  }

  // Trajectories:
  std::vector<Eigen::VectorXd> true_path;
  std::vector<Eigen::VectorXd> est_path;
  std::vector<Eigen::VectorXd> true_path2;
  std::vector<Eigen::VectorXd> est_path2;

  //GLVisualizer vis;
  //vis.SetCenterAndHeight(Eigen::Vector2d::Zero(), 6);

  // Prior on position
  Eigen::VectorXd x_TRUE(6);
  x_TRUE <<-2.0, 0.0, M_PI_2,2.0, 0.0, -M_PI_2;
  //Eigen::VectorXd x_TRUE2 = Eigen::Vector3d(-2.0, 0.0, M_PI_2);
  double std_x = 0.05;
  Eigen::VectorXd x_temp(6);
  x_temp<<std_x * dist(mt), std_x * dist(mt), std_x * dist(mt),std_x * dist(mt), std_x * dist(mt), std_x * dist(mt);
  Eigen::VectorXd x_t = x_TRUE + x_temp;
  //Eigen::VectorXd x_t2 = x_TRUE2 + Eigen::Vector3d(std_x * dist(mt), std_x * dist(mt), std_x * dist(mt));

  Eigen::MatrixXd Sigma_t = std_x * std_x * Eigen::MatrixXd::Identity(6,6);
  //Eigen::MatrixXd Sigma_t2 = std_x * std_x * Eigen::Matrix3d::Identity();
  Eigen::MatrixXd Sigma_temp;

  int len;
  int len2;

  true_path.push_back(x_TRUE);
  est_path.push_back(x_t);

  true_path2.push_back(x_TRUE.segment<3>(3));
  est_path2.push_back(x_t.segment<3>(3));

  // Noise Covariance:
  double std_n = 0.03;
  Eigen::MatrixXd Sigma_n = std_n * std_n * Eigen::MatrixXd::Identity(4,4);
  double std_m = 0.05;
  Eigen::Matrix2d Sigma_m = std_m * std_m * Eigen::MatrixXd::Identity(2,2);

  Eigen::VectorXd u(4);
  u<<0.2,-0.1,0.2,-0.1;
 // Eigen::VectorXd u2 = Eigen::Vector2d(0.2, -0.1);
  double dt = 0.1;
  for (size_t i = 0; i < 1000; ++i) {
    // Sample n:
    Eigen::VectorXd n(4);
    n<<std_n * dist(mt), std_n * dist(mt),  std_n * dist(mt),  std_n * dist(mt);
    //Eigen::VectorXd n2 = Eigen::Vector2d(std_n * dist(mt), std_n * dist(mt));
    // True Propagation (we'll assume w is not close to 0):
    Eigen::VectorXd u_tmp = n+u;
    //Eigen::VectorXd u_tmp2 = n2+u2;
    double th_old = x_TRUE[2];
    double th_old2 = x_TRUE[5];

    double th_new = th_old + dt * u_tmp[1];
    double th_new2 = th_old2 + dt * u_tmp[3];
    x_TRUE[0] += u_tmp[0]/u_tmp[1] * (std::sin(th_new) - std::sin(th_old));
    x_TRUE[1] += u_tmp[0]/u_tmp[1] * (-std::cos(th_new) + std::cos(th_old));
    x_TRUE[2] = th_new;

    x_TRUE[3] += u_tmp[2]/u_tmp[3] * (std::sin(th_new2) - std::sin(th_old2));
    x_TRUE[4] += u_tmp[2]/u_tmp[3] * (-std::cos(th_new2) + std::cos(th_old2));
    x_TRUE[5] = th_new2;

    // Run an EKFSLAMPropagation step
    {
      Eigen::VectorXd x_new;
      Eigen::MatrixXd Sigma_new;
      EKFSLAMPropagate(x_t, Sigma_t, u, Sigma_n, dt, x_new, Sigma_new);
      x_t = x_new;
      Sigma_t = Sigma_new;
    }

    // Run an update every 5 iterations
    if (i%5 == 0) {
      // Compute all the landmarks within 2m
      std::vector<Eigen::VectorXd> close_z;
      std::vector<Eigen::MatrixXd> close_Sigma_m;
      for (auto& lm : landmarks) {
        if ((lm.head<2>() - x_TRUE.head<2>()).norm() < 2.0) {
          // Sample m:
          Eigen::VectorXd m = Eigen::Vector2d(std_m * dist(mt), std_m * dist(mt));

          Eigen::Matrix2d C;
          C << std::cos(x_TRUE[2]), std::sin(x_TRUE[2]),
              -std::sin(x_TRUE[2]), std::cos(x_TRUE[2]);

          close_z.push_back(C * (lm.head<2>() - x_TRUE.head<2>()) + m);
          close_Sigma_m.push_back(Sigma_m);
        }
      }

     if (close_z.size() > 0) {
        Eigen::VectorXd x_new;
        Eigen::MatrixXd Sigma_new;

        int per = 0;
        EKFSLAMRelPosUpdate(x_t, Sigma_t, close_z, close_Sigma_m, x_new, Sigma_new,per);
        x_t = x_new;
        Sigma_t = Sigma_new;




      }
    }
      if (i%5 == 0) {
      // Compute all the landmarks within 2m
      std::vector<Eigen::VectorXd> close_z;
      std::vector<Eigen::MatrixXd> close_Sigma_m;
      for (auto& lm : landmarks) {
        if ((lm.head<2>() - x_TRUE.segment<2>(3)).norm() < 2.0) {
          // Sample m:
          Eigen::VectorXd m = Eigen::Vector2d(std_m * dist(mt), std_m * dist(mt));

          Eigen::Matrix2d C;
          C << std::cos(x_TRUE[5]), std::sin(x_TRUE[5]),
              -std::sin(x_TRUE[5]), std::cos(x_TRUE[5]);

          close_z.push_back(C * (lm.head<2>() - x_TRUE.segment<2>(3)) + m);
          close_Sigma_m.push_back(Sigma_m);
        }
      }

     if (close_z.size() > 0) {
        Eigen::VectorXd x_new;
        Eigen::MatrixXd Sigma_new;

        int per = 1;
        EKFSLAMRelPosUpdate(x_t, Sigma_t, close_z, close_Sigma_m, x_new, Sigma_new,per);
        x_t = x_new;
        Sigma_t = Sigma_new;




      }
    }

   /* // Draw the current state
    vis.ClearTempLines();
    // Draw the trajectory and the estimated trajectory
    true_path.push_back(x_TRUE);
    est_path.push_back(x_t);
    vis.AddTempLine(est_path, Color::RED, 1.5);
    vis.AddTempLine(true_path, Color::BLUE, 1.5);
    // Draw the uncertainty of the robot
    vis.AddTempEllipse(x_t.head<2>(), Sigma_t.topLeftCorner<2,2>(), Color::RED, 1.2);

    // Draw the uncertainty of all the landmarks in the state
    for (size_t j = 3; j < x_t.size(); j+=2) {
      vis.AddTempEllipse(x_t.segment<2>(j), Sigma_t.block<2,2>(j,j), Color::RED, 1.2);
    }

    // Draw the true landmark positions
    for (size_t j = 0; j < landmarks.size(); ++j) {
      vis.AddTempEllipse(landmarks[j].head<2>(), 0.0001 * Eigen::Matrix2d::Identity(), Color::BLUE, 2);
    }

    //vis.UpdateLines();

  // Run an EKFSLAMPropagation step
    {
      Eigen::VectorXd x_new;
      Eigen::MatrixXd Sigma_new;
      EKFSLAMPropagate(x_t2, Sigma_t2, u2, Sigma_n, dt, x_new, Sigma_new);
      x_t2 = x_new;
      Sigma_t2 = Sigma_new;
    }

// Run an update every 5 iterations
    if (i%5 == 0) {
      // Compute all the landmarks within 2m
      std::vector<Eigen::VectorXd> close_z;
      std::vector<Eigen::MatrixXd> close_Sigma_m;
      for (auto& lm : landmarks) {
        if ((lm.head<2>() - x_TRUE2.head<2>()).norm() < 2.0) {
          // Sample m:
          Eigen::VectorXd m = Eigen::Vector2d(std_m * dist(mt), std_m * dist(mt));

          Eigen::Matrix2d C;
          C << std::cos(x_TRUE2[2]), std::sin(x_TRUE2[2]),
              -std::sin(x_TRUE2[2]), std::cos(x_TRUE2[2]);

          close_z.push_back(C * (lm.head<2>() - x_TRUE2.head<2>()) + m);
          close_Sigma_m.push_back(Sigma_m);
        }
      }

      if (close_z.size() > 0) {
        Eigen::VectorXd x_new;
        Eigen::MatrixXd Sigma_new;
        EKFSLAMRelPosUpdate(x_t2, Sigma_t2, close_z, close_Sigma_m, x_new, Sigma_new);
        x_t2 = x_new;
        Sigma_t2 = Sigma_new;
        len = x_t2.size();
        len2 = x_t.size();
        x_temp = x_t2;
        x_temp.head(3) = x_t.head(3);
        x_t = x_temp;

        Sigma_temp = Sigma_t2;
       // Sigma_temp.block(0,0,3,len) = Eigen::MatrixXd::Zero(3,len);
        //Sigma_temp.block(0,0,len,3) = Eigen::MatrixXd::Zero(len,3);
        Sigma_temp.block(0,0,len2,3) = Sigma_t.block(0,0,len2,3);
        Sigma_temp.block(0,0,3,len2) = Sigma_t.block(0,0,3,len2);
        Sigma_t = Sigma_temp;



      }
    }*/


    // Draw the current state
   /* vis.ClearTempLines();
    // Draw the trajectory and the estimated trajectory
    true_path.push_back(x_TRUE);
    est_path.push_back(x_t);
    true_path2.push_back(x_TRUE.segment<3>(3));
    est_path2.push_back(x_t.segment<3>(3));

    vis.AddTempLine(est_path, Color::RED, 1.5);
    vis.AddTempLine(true_path, Color::BLUE, 1.5);

    vis.AddTempLine(est_path2, Color::GREEN, 1.5);
    vis.AddTempLine(true_path2, Color::YELLOW, 1.5);
    // Draw the uncertainty of the robot
    vis.AddTempEllipse(x_t.head<2>(), Sigma_t.topLeftCorner<2,2>(), Color::RED, 1.2);
    vis.AddTempEllipse(x_t.segment<2>(3), Sigma_t.block<2,2>(3,3), Color::RED, 1.2);
    // Draw the uncertainty of all the landmarks in the state
    for (size_t j = 6; j < x_t.size(); j+=2) {
      vis.AddTempEllipse(x_t.segment<2>(j), Sigma_t.block<2,2>(j,j), Color::RED, 1.2);
    }

    // Draw the true landmark positions
    for (size_t j = 0; j < landmarks.size(); ++j) {
      vis.AddTempEllipse(landmarks[j].head<2>(), 0.0001 * Eigen::Matrix2d::Identity(), Color::BLUE, 2);
    }

    vis.UpdateLines();

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  vis.PauseFigure(-1);*/
}}

int main(int argc, char** argv) {

  {
    // specific node name and handle
    ros::init(argc, argv,"slam");
    ros::NodeHandle nh;

    // subscribe to the feature and commanded velocity topics
    //ros::Subscriber tb3_1_featureSub = nh.subscribe("/tb3_1/features", 100);
    // ros::Subscriber tb3_0_inputSub = nh.subscribe("/tb3_0/cmd_vel", 100);
    // ros::Subscriber tb3_1_inputSub = nh.subscribe("/tb3_0/cmd_vel", 100);

    // Now generate a new scene for SLAM and try to Update estimates through it
    TestRelPosSLAM();

    // run until stopped
    // ros::spin();
  }
}
