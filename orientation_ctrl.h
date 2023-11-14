// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_example_controllers/compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <iostream>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/kinfam_io.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>

#include <vector>
#include <utility>
#include <math.h>
#include <time.h>
#include <Eigen/QR>
#include <Eigen/Dense>
#include <array>
#include <functional>

namespace franka_example_controllers {

    class OrientationControl : public controller_interface::MultiInterfaceController<
        franka_hw::FrankaModelInterface,
        hardware_interface::EffortJointInterface,
        franka_hw::FrankaStateInterface> {
    public:
        bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
        void starting(const ros::Time&) override;
        void update(const ros::Time&, const ros::Duration& period) override;

    private:
        // Saturation
        Eigen::Matrix<double, 7, 1> saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated, const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

        std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
        std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
        std::vector<hardware_interface::JointHandle> joint_handles_;

        double filter_params_{ 0.9999/*0.005*/ };
        double nullspace_stiffness_{ 20.0 };
        double nullspace_stiffness_target_{ 20.0 };
        const double delta_tau_max_{ 20.0 };
        Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
        Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
        Eigen::Matrix<double, 6, 6> cartesian_damping_;
        Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
        Eigen::Matrix<double, 7, 1> q_d_nullspace_;
        Eigen::Vector3d position_d_;
        Eigen::Quaterniond orientation_d_;
        Eigen::Vector3d position_d_target_;
        Eigen::Quaterniond orientation_d_target_;

	ros::Publisher update_cost_pub;
	ros::Publisher cost_pub;
	ros::Publisher rpy_msr1_pub;
	ros::Publisher rpy_msr2_pub;
        ros::Publisher franka_EE_pose_pub;
        ros::Publisher franka_EE_velocity_pub;
        ros::Publisher franka_EE_wrench_pub;
	ros::Publisher franka_q_pose_pub;
        ros::Publisher franka_q_velocity_pub;
        int cont_task_setpoint;

        double msrTimestep;
        double filtTime;
        double digfilt;
        std::array<double, 7> dq_filt;
        
        double err_f_x;
        double err_f_x_old;
        double err_f_y;
        double err_f_y_old;
        double err_f_z;
        double err_f_z_old;
        double norm_external_f_old;
        double Kgd;
        double grad_err_f_pos0;
        double grad_err_f_pos1;
        double rpy_gp_1;
        double rpy_gp_2;
        
        bool stop_update;
        int cont_stop;

        Eigen::Vector3d position_old, dposition, rpy_old, rpy_msr_old, drpy, rpy_cmd, rpy_cmd_filt, rpy_msr, drpy_cmd, dw_psp_world, drpy_filt, dposition_filt;
        Eigen::Vector3d rpy_init, vel_imp_t, pos_imp_t, vel_imp_r, pos_imp_r, w_psp_world;
	Eigen::Vector3d position_init;
	Eigen::Matrix<double, 6, 6> Kpos;
	Eigen::Matrix<double, 7, 7> Kp, Kd, Ki;
	Eigen::Matrix<double, 7, 1> joint_velocity_d, joint_pos_d;
        Eigen::Matrix<double, 7, 1> int_jnt_pos_err;
        Eigen::Matrix<double, 7, 1> dq_filt_Eigen;
        Eigen::Matrix3d R_cmd;
        
        double rpy_gp_update;
        Eigen::Vector3d rpy_gp;
        bool rand_step;
        double Kpgd;

        double th_tau_msr;
	bool initialization;

        double h_damp_t;
        double h_damp_r;
	double damping_importato;
	double damping_old;
        double mass_imp;
        double inertia_imp;
        double translational_stiffness;
        double rotational_stiffness;
	double stiffness_importata;

        franka::RobotState initial_state;
        
        ros::Subscriber sub_rpy_gp_update;
        void rpy_gp_updateCallback(const std_msgs::Float64::ConstPtr& msg_BOupdate);
        ros::Subscriber sub_cmd_rot_1;
        void sub_cmd_rot_1Callback(const std_msgs::Float64::ConstPtr& msg1);
        ros::Subscriber sub_cmd_rot_2;
        void sub_cmd_rot_2Callback(const std_msgs::Float64::ConstPtr& msg2);
    };

}  // namespace franka_example_controllers#pragma once

