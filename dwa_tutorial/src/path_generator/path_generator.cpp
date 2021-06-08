#include <iostream>
#include <nav_msgs/Odometry.h>

#include "dwa_tutorial/DWACalculationData.h"
#include "dwa_tutorial/dynamic_window_approach.h"

class PathGenerator {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher pub_cmd_vel_;
    ros::Subscriber sub_data_;
    ros::Subscriber sub_odom_;
    PathPlan::DynamicWindowApproach dwa_;
    geometry_msgs::Twist curt_vel_;

    void setParameters ( ) {
        std::string target_frame = pnh_.param<std::string>( "target_frame", "base_footprint" );
        dwa_.setTargetFrame( target_frame );
        int predict_step = pnh_.param<int>( "predict_step", 30 );
        double sampling_time = pnh_.param<double>( "sampling_time", 0.1 );
        double vel_step = pnh_.param<double>( "velocity_step", 7.0 );
        double ang_vel_step = pnh_.param<double>( "angle_velocity_step", 15.0 );
        dwa_.setStepValue( predict_step, sampling_time, vel_step, ang_vel_step );
        std::vector<double> ang_range = pnh_.param<std::vector<double>>( "angle_range", { -1.0, 1.0 } );
        double max_acc = pnh_.param<double>( "max_acceration", 1.5 );
        double max_ang_acc = pnh_.param<double>( "max_ang_acceration", 1.5 );
        double acc_gain = pnh_.param<double>( "acceleration_gain", 1.1 );
        dwa_.setVelocityLimit( ang_range, max_acc, max_ang_acc, acc_gain );
        double goal = pnh_.param<double>( "weight_goal", 1.0 );
        double obs = pnh_.param<double>( "weight_obstacle", 1.0 );
        double ang = pnh_.param<double>( "weight_angle", 2.0 );
        double vel = pnh_.param<double>( "weight_velocity", 1.0 );
        dwa_.setWeight( goal, obs, ang, vel );
        double obs_cnt_rad = pnh_.param<double>( "obstacle_center_radius", 0.35 );
        double obs_pts_rad = pnh_.param<double>( "obstacle_points_radius", 0.35 );
        dwa_.setCostDistance ( obs_cnt_rad, obs_pts_rad );
        bool is_display_path = pnh_.param<bool>( "is_display_path", false );
        bool is_display_all_path = pnh_.param<bool>( "is_display_all_path", false );
        dwa_.setDisplayFlag( is_display_path, is_display_all_path );
        std::cout << "\n========================================\n[ path generator parameters ]"
            << "\n  * target_frame : " << target_frame
            << "\n\n< DWA : StepValue >"
            << "\n  * predict_step : " << predict_step
            << "\n  * sampling_time : " << sampling_time
            << "\n  * velocity_step : " << vel_step
            << "\n  * angle_velocity_step : " << ang_vel_step
            << "\n\n< DWA : VelocityLimit >"
            << "\n  * angle_range : " << ang_range[0] << ", " << ang_range[1]
            << "\n  * max_acceration : " << max_acc
            << "\n  * max_ang_acceration : " << max_acc
            << "\n  * acceleration_gain : " << max_acc
            << "\n\n< DWA : Weight >"
            << "\n  * weight_goal : " << goal
            << "\n  * weight_obstacle : " << obs
            << "\n  * weight_angle : " << ang
            << "\n  * weight_velocity : " << vel
            << "\n\n< DWA : CostDistance >"
            << "\n  * obstacle_center_radius : " << obs_cnt_rad
            << "\n  * obstacle_points_radius : " << obs_pts_rad
            << "\n\n< DWA : DisplayFlag >"
            << "\n  * is_display_path : " << ( is_display_path ? "True" : "False" )
            << "\n  * is_display_all_path : " << ( is_display_all_path ? "True" : "False" )
            << "\n========================================\n"
        << std::endl;
    }

    void callbackDWACalculationData  ( const dwa_tutorial::DWACalculationDataConstPtr &input_data ) { 
        dwa_tutorial::DWACalculationData data = *input_data; 
        geometry_msgs::Twist vel;
        dwa_.generatePath2Target ( curt_vel_, data, &vel );
        pub_cmd_vel_.publish( vel );
        ROS_INFO( "\n< Target > \nx : %.3f [m]\ny : %.3f [m]\nangle : %.3f[rad]\n\
        \n< Path > \nLinear  : %.3f [m/s]\nAngular : %.3f [rad/s]\n",
        data.target.x, data.target.y, data.target_angle, vel.linear.x , vel.angular.z );
    }

    void callbackOdometry ( const nav_msgs::OdometryConstPtr &odom_msg ) { curt_vel_ = odom_msg->twist.twist; }


public:
    PathGenerator() : nh_(), pnh_("~") {
        setParameters ( );
        // Subscriber :
        sub_data_ = nh_.subscribe( "/dwa_calculation_data", 1, &PathGenerator::callbackDWACalculationData, this );
        sub_odom_ = nh_.subscribe( "/odom", 1, &PathGenerator::callbackOdometry, this );
        // Publisher :
        pub_cmd_vel_ = nh_.advertise< geometry_msgs::Twist >( "/cmd_vel_mux/input/teleop", 1 );
    }
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "path_generator");
	PathGenerator pg;
    ros::spin();
}