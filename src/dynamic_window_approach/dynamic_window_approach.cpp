#include <local_path_planning_tutorial/dynamic_window_approach.hpp>

using namespace local_path_planning_tutorial;

void DynamicWindowApproach::displayOptimalPathMarker ( double optimal_vel, double optimal_ang_vel ) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = dwap_->target_frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = "optimal_path";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.03;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(0.1);

    int predict_step = dwap_->predict_step;
    double theta = optimal_ang_vel;
    double sampling_time = dwap_->sampling_time;
    geometry_msgs::Point pt, pre_pt;
    pt.z = 0.1;
    for ( int step = 0; step < predict_step; ++step) {
        pre_pt = pt;
        pt.x = optimal_vel * cos(theta) * sampling_time + pre_pt.x;
        pt.y = optimal_vel * sin(theta) * sampling_time + pre_pt.y;
        theta = optimal_ang_vel * sampling_time + theta;
        if ( std::hypotf( pt.x, pt.y ) > 1.5 ) break;
        marker.points.push_back( pt );
    }
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    pub_path_marker_.publish ( marker );
}

void DynamicWindowApproach::displayAllPathMarker ( std::vector< EvaluatedPath >& path_list ) {
    int marker_num = 0;
    visualization_msgs::MarkerArray marker_all;
    std::string target_frame = dwap_->target_frame;
    int predict_step = dwap_->predict_step;
    double sampling_time = dwap_->sampling_time;

    for( auto path : path_list ) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = target_frame;
        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.ns = "all_path";
        marker.id = marker_num;
        marker.scale.x = 0.03;
        marker.color.a = 0.5;
        marker.lifetime = ros::Duration(0.1);
        if ( path.is_collision ) {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        } else {
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
        }
        double theta = path.ang_vel;
        geometry_msgs::Point pt, pre_pt;
        pt.z = 0.1;
        for ( int step = 0; step < predict_step; ++step) {
            pre_pt = pt;
            pt.x = path.vel * cos(theta) * sampling_time + pre_pt.x;
            pt.y = path.vel * sin(theta) * sampling_time + pre_pt.y;
            theta = path.ang_vel * sampling_time + theta;
            if ( std::hypotf( pt.x, pt.y ) > 1.0 ) break;
            marker.points.push_back( pt );
        }
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker_num++;
        marker_all.markers.push_back( marker );
    }
    pub_path_marker_all_.publish( marker_all );
}

DynamicWindowApproach::DynamicWindowApproach ( ) : nh_(), pnh_("~") {
    pub_path_marker_ = nh_.advertise< visualization_msgs::Marker >( "/dwa_path_marker", 1 );
    pub_path_marker_all_ = nh_.advertise< visualization_msgs::MarkerArray >( "/dwa_path_marker_all", 1 );
    dwap_.reset ( new DWAParameters );
    // DWA Parameters :
	setTargetFrame( pnh_.param<std::string>( "base_footprint_name", "base_footprint" ) );
	setStepValue( pnh_.param<int>( "predict_step", 30 ), pnh_.param<double>( "sampling_time", 0.1 ), pnh_.param<double>( "velocity_step", 7.0 ), pnh_.param<double>( "angle_velocity_step", 15.0 ) );
	setVelocityLimit( pnh_.param<std::vector<double>>( "linear_range", { 0.2, 1.2 } ), pnh_.param<std::vector<double>>( "angle_range", { -1.0, 1.0 } ), pnh_.param<double>( "max_acceration", 1.5 ), pnh_.param<double>( "max_ang_acceration", 1.5 ), pnh_.param<double>( "acceleration_gain", 1.1 ) );
	setWeight( pnh_.param<double>( "weight_goal", 1.0 ), pnh_.param<double>( "weight_obstacle", 1.0 ), pnh_.param<double>( "weight_angle", 2.0 ), pnh_.param<double>( "weight_velocity", 1.0 ) );
	setCostDistance ( pnh_.param<double>( "obstacle_cost_radius", 0.35 ) );
	setDisplayFlag( pnh_.param<bool>( "is_display_path", false ), pnh_.param<bool>( "is_display_all_path", false ) );

	std::cout << "\n========================================\n[ path generator parameters ]"
		<< "\n  * base_footprint_name    : " << pnh_.param<std::string>( "base_footprint_name", "base_footprint" )
		<< "\n\n< DWA : StepValue >"
		<< "\n  * predict_step           : " << pnh_.param<int>( "predict_step", 30 )
		<< "\n  * sampling_time          : " << pnh_.param<double>( "sampling_time", 0.1 )
		<< "\n  * velocity_step          : " << pnh_.param<double>( "velocity_step", 7.0 )
		<< "\n  * angle_velocity_step    : " << pnh_.param<double>( "angle_velocity_step", 15.0 )
		<< "\n\n< DWA : VelocityLimit >"
        << "\n  * linear_range           : " << pnh_.param<std::vector<double>>( "linear_range", { 0.2, 1.2 } )[0] << ", " << pnh_.param<std::vector<double>>( "linear_range", { 0.2, 1.2 } )[1]
		<< "\n  * angle_range            : " << pnh_.param<std::vector<double>>( "angle_range", { -1.0, 1.0 } )[0] << ", " << pnh_.param<std::vector<double>>( "angle_range", { -1.0, 1.0 } )[1]
		<< "\n  * max_acceration         : " << pnh_.param<double>( "max_acceration", 1.5 )
		<< "\n  * max_ang_acceration     : " << pnh_.param<double>( "max_ang_acceration", 1.5 )
		<< "\n  * acceleration_gain      : " << pnh_.param<double>( "acceleration_gain", 1.1 )
		<< "\n\n< DWA : Weight >"
		<< "\n  * weight_goal            : " << pnh_.param<double>( "weight_goal", 1.0 )
		<< "\n  * weight_obstacle        : " << pnh_.param<double>( "weight_obstacle", 1.0 )
		<< "\n  * weight_angle           : " << pnh_.param<double>( "weight_angle", 2.0 )
		<< "\n  * weight_velocity        : " << pnh_.param<double>( "weight_velocity", 1.0 )
		<< "\n\n< DWA : CostDistance >"
		<< "\n  * obstacle_cost_radius   : " << pnh_.param<double>( "obstacle_cost_radius", 0.35 )
		<< "\n\n< DWA : DisplayFlag >"
		<< "\n  * is_display_path        : " << ( pnh_.param<bool>( "is_display_path", false ) ? "True" : "False" )
		<< "\n  * is_display_all_path    : " << ( pnh_.param<bool>( "is_display_all_path", false ) ? "True" : "False" )
		<< "\n========================================\n"
	<< std::endl;
}

bool DynamicWindowApproach::generatePath2Target ( 
    const geometry_msgs::Point& target,
    const PointCloud::Ptr obstacles,
    geometry_msgs::TwistPtr output_path ) 
{
    // ?????????????????????????????????????????????
    std::vector< EvaluatedPath > path_list;
    MinMaxValue mmv;

    std::vector<double> lin_vels = dwap_->lin_vels;
    std::vector<double> ang_vels = dwap_->ang_vels;
    int predict_step = dwap_->predict_step;
    double sampling_time = dwap_->sampling_time;

    std::vector<int> k_indices;
    std::vector<float> k_distances;
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud (obstacles);
    double dist_nearest_obstacle = 0.0;
    double obstacle_cost_radius = dwap_->obstacle_cost_radius;

    double g = dwap_->weight_goal;
    double o = dwap_->weight_obs;
    double a = dwap_->weight_ang;
    double v = dwap_->weight_vel;
    double max_score = 0.0;
    double optimal_vel = 0.0;
    double optimal_ang_vel = 0.0;
    bool exists_path = false;

    // ????????????
    for ( auto& vel : lin_vels ) {
        for ( auto& ang_vel : ang_vels ) {
            EvaluatedPath eval;
            eval.vel = vel;
            eval.ang_vel = ang_vel;

            Path path, pre_path; 
            bool is_collision = false;
            double theta = ang_vel;
            // ??????????????????
            for ( int step = 0; step < predict_step; ++step) {
                pre_path = path;
                path.point.x = vel * cos(theta) * sampling_time + pre_path.point.x;
                path.point.y = vel * sin(theta) * sampling_time + pre_path.point.y;
                path.theta = ang_vel * sampling_time + pre_path.theta;
                if ( kdtree.nearestKSearch ( path.point, 1, k_indices, k_distances ) > 0 ) {
                    dist_nearest_obstacle = std::sqrt(k_distances[0]);
                    if ( dist_nearest_obstacle <= obstacle_cost_radius ) {
                        is_collision = true;
                        break;
                    }
                } else {
                    ROS_ERROR("No Obstacle");
                }
                theta = path.theta;
            }
            eval.is_collision = is_collision;
            if ( !is_collision ) {
                // [ dist(v,??) ]  Evaluate the distance between the predicted point and the nearest obstacle :
                eval.obs.score = dist_nearest_obstacle;
                // [ goal(v,??) ]  Evaluate the distance between the predicted point and Taget point  :
                eval.goal.score = 1 / (1.0 + std::hypotf(  target.x - path.point.x, target.y - path.point.y ));
                // [ heading(v,??) ] Evaluate the angle of difference between predicted point and Target point :
                double ang_target2path = std::fabs( std::atan2 ( target.y - path.point.y, target.x - path.point.x ) );
                double ang_diff = std::fabs( std::atan2 ( target.y, target.x ) - path.theta );
                eval.angle.score = 2 * M_PI - ( ang_target2path + ang_diff); 
                // [ velocity(v,??) ] Evaluate Velocity :
                eval.vel_s.score = std::fabs( vel );
                // Update the max and min evaluation values ??????of all paths :
                if( mmv.obs.min_score > eval.obs.score ) mmv.obs.min_score = eval.obs.score;
                if( mmv.obs.max_score < eval.obs.score ) mmv.obs.max_score = eval.obs.score;
                if( mmv.goal.min_score > eval.goal.score ) mmv.goal.min_score = eval.goal.score;
                if( mmv.goal.max_score < eval.goal.score ) mmv.goal.max_score = eval.goal.score;
                if( mmv.angle.min_score > eval.angle.score ) mmv.angle.min_score = eval.angle.score;
                if( mmv.angle.max_score < eval.angle.score ) mmv.angle.max_score = eval.angle.score;
                if( mmv.vel.min_score > eval.vel_s.score ) mmv.vel.min_score = eval.vel_s.score;
                if( mmv.vel.max_score < eval.vel_s.score ) mmv.vel.max_score = eval.vel_s.score;
            }
            path_list.push_back( eval );
        }
    }

    // ??????????????????????????????????????????????????????
    for ( auto& path : path_list ) {
        if ( path.is_collision || ( path.vel == 0.0 && path.ang_vel == 0.0 ) ) continue;
        if ( mmv.goal.max_score - mmv.goal.min_score != 0.0 ) {
            path.goal.norm_score = ( path.goal.score - mmv.goal.min_score ) / ( mmv.goal.max_score - mmv.goal.min_score );
        } else path.goal.norm_score = 0.0;
        if ( mmv.obs.max_score - mmv.obs.min_score != 0.0 ) {
            path.obs.norm_score = ( path.obs.score - mmv.obs.min_score ) / ( mmv.obs.max_score - mmv.obs.min_score );
        } else path.obs.norm_score = 0.0;
        if ( mmv.angle.max_score - mmv.angle.min_score != 0.0 ) {
            path.angle.norm_score = ( path.angle.score - mmv.angle.min_score ) / ( mmv.angle.max_score - mmv.angle.min_score );
        } else path.angle.norm_score = 0.0;
        if ( mmv.vel.max_score - mmv.vel.min_score != 0.0 ) {
            path.vel_s.norm_score = ( path.vel_s.score - mmv.vel.min_score ) / ( mmv.vel.max_score - mmv.vel.min_score );
        } else path.vel_s.norm_score = 0.0; 

        double sum_score = 	  g * path.goal.norm_score 
                            + o * path.obs.norm_score
                            + a * path.angle.norm_score
                            + v * path.vel_s.norm_score;
        if ( sum_score > max_score ) {
            max_score = sum_score;
            optimal_vel = path.vel;
            optimal_ang_vel = path.ang_vel;
            exists_path = true;
        }		
    }

    // ?????????????????????????????????
    if ( exists_path ) {
        output_path->linear.x = optimal_vel;
        output_path->angular.z = optimal_ang_vel;
        std::cout << "\n[Velocity]"  
        << "\n* linear  [m/s]   : " <<  optimal_vel 
        << "\n* angular [rad/s] : " <<  optimal_ang_vel << std::endl;
    } else {
        ROS_ERROR("No Optimal Path");
    }

    if ( is_disp_path_ ) displayOptimalPathMarker ( optimal_vel, optimal_ang_vel );
    if ( is_disp_all_path_ ) displayAllPathMarker ( path_list );
    return exists_path;
}