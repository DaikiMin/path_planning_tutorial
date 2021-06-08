#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include<potential_method/potential_method.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class MySubscribers {
    protected : 
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        tf::TransformListener tf_listener_;
        ros::Subscriber sub_odom_;
        ros::Subscriber sub_sensor_;
        ros::Subscriber sub_tgt_pose_;
        
        geometry_msgs::Twist curt_vel_;
        geometry_msgs::Point rob_pt_map_;
        geometry_msgs::Point tgt_pt_map_;
        geometry_msgs::Point tgt_pt_base_;
        PointCloud::Ptr obs_map_;
        PointCloud::Ptr obs_base_;

        visualization_msgs::Marker path_mrk_;

        bool is_odom_;
        bool is_sensor_;

        geometry_msgs::Point transformPoint ( std::string org_frame, std::string target_frame, geometry_msgs::Point point ) {
			geometry_msgs::PointStamped pt_transformed;
			geometry_msgs::PointStamped pt;
			pt.header.frame_id = org_frame;
			pt.header.stamp = ros::Time(0);
			pt.point = point;
			if ( tf_listener_.frameExists( target_frame ) ) {
				try {
					tf_listener_.transformPoint( target_frame, pt, pt_transformed );
				} catch ( tf::TransformException ex ) {
					ROS_ERROR( "%s",ex.what( ) );
				}
			} else {
				ROS_ERROR("target_frame is not Exists");
			}
			return pt_transformed.point;
		}

        void callbackOdometry ( const nav_msgs::OdometryConstPtr &odom_msg ) { 
            curt_vel_ = odom_msg->twist.twist; 
            rob_pt_map_ = odom_msg->pose.pose.position;
            is_odom_ = true;
        }
        void callbackSensor( const sensor_msgs::PointCloud2ConstPtr &sensor_msg ) {
            PointCloud cloud_src;
            PointCloud cloud_transformed;
            pcl::fromROSMsg<PointT>( *sensor_msg, cloud_src );
            *obs_map_ = cloud_src;
            obs_map_->header.frame_id = "map";
            try {
                // transform frame :
                tf_listener_.waitForTransform("base_footprint", "map", ros::Time(0), ros::Duration(1.0));
                pcl_ros::transformPointCloud("base_footprint", ros::Time(0), cloud_src, "map",  cloud_transformed, tf_listener_);
                
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s", ex.what());
            }
            *obs_base_ = cloud_transformed;
            obs_base_->header.frame_id = "base_footprint";
            is_sensor_ = true;
        }
        void callbackTarget ( const geometry_msgs::PoseStampedConstPtr &msg ) {
            tgt_pt_map_ = msg->pose.position;
            tgt_pt_base_ = transformPoint( "map", "base_footprint", tgt_pt_map_ );
            path_mrk_.points.clear();
        }

    public : 
        MySubscribers() : nh_(), pnh_("~") {
            sub_odom_ = nh_.subscribe( "/odom", 1, &MySubscribers::callbackOdometry, this );
            sub_sensor_ = nh_.subscribe( "/sensor", 1 , &MySubscribers::callbackSensor, this );
            sub_tgt_pose_ = nh_.subscribe( "/move_base_simple/goal", 10, &MySubscribers::callbackTarget, this );
            obs_map_.reset( new PointCloud() );
            obs_base_.reset( new PointCloud() );
            is_odom_ = false;
            is_sensor_ = false;


            path_mrk_.header.frame_id = "map";
            path_mrk_.header.stamp = ros::Time::now();
            path_mrk_.ns = "optimal_path";
            path_mrk_.id = 1;
            path_mrk_.type = visualization_msgs::Marker::LINE_STRIP;
            path_mrk_.action = visualization_msgs::Marker::ADD;
            path_mrk_.lifetime = ros::Duration(1.0);
            path_mrk_.scale.x = 0.05;
            path_mrk_.color.a = 1.0;
            path_mrk_.color.r = 0.0;
            path_mrk_.color.g = 1.0;
            path_mrk_.color.b = 0.0;
        }

};

class PathGenerator : private MySubscribers {
    private :
        ros::Publisher pub_cmd_vel_;
        ros::Publisher pub_path_marker_;

    public:
        PathGenerator() {
            pub_cmd_vel_ = nh_.advertise< geometry_msgs::Twist >( "/cmd_vel_mux/input/teleop", 1 );
            pub_path_marker_ = nh_.advertise< visualization_msgs::Marker >( "/path", 1 );
            tgt_pt_map_.x = 0.0;
            tgt_pt_map_.y = 0.0;
        }

        void generatePath() {
            Potential::PotentialMethod pot;
            pot.setWeightObstacle( pnh_.param<double>( "weight_obstacle", 1.0 ) );
            pot.setWeightGoal( pnh_.param<double>( "weight_goal", 1.0 ) );
            pot.setDelta( pnh_.param<double>( "delta", 0.1 ) );
            pot.setGridWidth(  pnh_.param<double>( "grid_width", 0.25 ) );

            ros::Rate rate(15);
            while(ros::ok()){
                if( is_odom_ && is_sensor_ ) break;
                ros::spinOnce();
                rate.sleep();
            }
            while(ros::ok()){
                geometry_msgs::Twist path;
                double dist = std::hypotf(tgt_pt_base_.x, tgt_pt_base_.y );
                double ang = std::atan2( tgt_pt_base_.y, tgt_pt_base_.x );
                tgt_pt_base_ = transformPoint( "map", "base_footprint", tgt_pt_map_ );
                pot.potentialField( tgt_pt_map_, obs_map_ );
                if( dist > 0.1 ) pot.generatePath( tgt_pt_base_, obs_base_, curt_vel_, &path );
                std::cout   << "\n[ Pos ]"
                            << "\nrobot_map  = ( " << rob_pt_map_.x << ", " << rob_pt_map_.y << " )"
                            << "\ntarget_map = ( " << tgt_pt_map_.x << ", " << tgt_pt_map_.y << " )"
                            << "\ntarget_base = ( " << tgt_pt_base_.x << ", " << tgt_pt_base_.y << " )"
                            << "\n[info]" 
                            << "\n dist = " << dist
                            << "\n ang = " << ang
                            << "\n[ Path ]"
                            << "\nlin = " << path.linear.x
                            << "\nang = " << path.angular.z  << std::endl;

                geometry_msgs::Point temp;
                temp.x = rob_pt_map_.x;
                temp.y = rob_pt_map_.y;
                temp.z = 0.1;
                path_mrk_.points.push_back( temp );
                path_mrk_.header.stamp = ros::Time::now();
                pub_path_marker_.publish ( path_mrk_ );
                pub_cmd_vel_.publish( path );
                ros::spinOnce();
                rate.sleep();
            }
        }
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "path_generator");
    PathGenerator pg;
    pg.generatePath();
    ros::spin();
}