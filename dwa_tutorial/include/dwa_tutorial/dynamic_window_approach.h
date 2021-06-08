#ifndef DYNAMIC_WINDOW_APPROACH
#define DYNAMIC_WINDOW_APPROACH

#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
#include <dwa_tutorial/DWACalculationData.h>

typedef struct {         
	double score;
	double norm_score;
} Score;
typedef struct {         
	double min_score = DBL_MAX;
    double max_score = DBL_MIN;
} MinMaxScore;
typedef struct {        
	Score goal;
	Score obs;
    Score angle; 
    Score vel;
	bool is_collision; 
} EvaluatedValue;
typedef struct {        
	MinMaxScore goal;
	MinMaxScore obs;
    MinMaxScore angle; 
    MinMaxScore vel;
}  MinMaxValue;
typedef struct {         
    double next_x; 
    double next_y;
	double next_theta;
} Path;
typedef struct {
	std::vector< Path > paths;
	EvaluatedValue score;
	double vel;
	double ang_vel;
} EvaluatedPath;
typedef struct {
	double min_vel;
	double max_vel;
	double delta_vel;
	double curt_vel;
	double curt_ang_vel;
	int pre_step;
	double dt;
	double cost_obs_cnt_rad;
	double cost_obs_pt_rad;
} DWAParameters;

namespace PathPlan {
    class DynamicWindowApproach {
	private :
		ros::NodeHandle nh_;
		ros::Publisher pub_path_marker_;
		ros::Publisher pub_path_marker_all_;

		bool is_disp_path_;
		bool is_disp_all_path_;

		std::string target_frame_;

		std::vector<double> ang_range_;
		std::vector<double> ang_vels_;
		double max_acc_;
		double max_ang_acc_;
		double acc_gain_;

		int predict_step_;
		double sampling_time_;
		double vel_step_;
		double ang_vel_step_;

		double obstacle_center_radius_;
		double obstacle_points_radius_;

		double weight_goal_;
		double weight_obs_;
		double weight_ang_;
		double weight_vel_;

		dwa_tutorial::DWACalculationData data_;

		bool setDWAParameters ( const geometry_msgs::Twist& curt_vel, const double tgt_dist, DWAParameters* dwa_param );
		void makePathsList( const DWAParameters& dwa_param, std::vector< EvaluatedPath >* paths_list );
		bool predictPath( const double vel, const double ang_vel, const DWAParameters& dwa_param, EvaluatedPath* paths, MinMaxValue* mmv );
		void evaluatePath ( const double dist_score, const double vel, const Path& base_path, EvaluatedValue* score, MinMaxValue* mmv );
		void normalizeEvaluationValues( std::vector< EvaluatedPath > *paths_list, const MinMaxValue& mmv );
		void displayOptimalPathMarker ( const std::vector< Path >& paths );
		void displayAllPathMarker ( std::vector< EvaluatedPath >& paths_list );
	public :
		DynamicWindowApproach();
		void setTargetFrame ( const std::string& target_frame );
		void setVelocityLimit ( const std::vector<double>& ang_range, const double max_acc, const double max_ang_acc, const double acc_gain );
		void setWeight ( const double goal, const double obs, const double ang, const double vel );
		void setCostDistance ( const double obstacle_center_radius, const double obstacle_points_radius );
		void setStepValue ( const int predict_step, const double sampling_time, const double vel_step, const double ang_vel_step );
		void setDisplayFlag ( const bool is_display_path, const bool is_display_all_path );
		bool generatePath2Target ( const geometry_msgs::Twist& curt_vel, const dwa_tutorial::DWACalculationData& data, geometry_msgs::Twist* output_path );
	};
}

#endif