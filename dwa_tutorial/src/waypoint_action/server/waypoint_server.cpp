#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <dwa_tutorial/WayPointDWAAction.h>
#include <dwa_tutorial/dwa_calculation_data_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class RobotSubscriber {
    private :
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Subscriber sub_sensor_;
        ros::Subscriber sub_vel_;
        tf::TransformListener tf_listener_;

        void sensorCb( const sensor_msgs::PointCloud2ConstPtr& sensor_msg ) { 
            PointCloud cloud_src;  
            pcl::fromROSMsg<PointT>( *sensor_msg, cloud_src ); 
            *cloud_obstacles_ = cloud_src;
        }
        void velocityCb( const geometry_msgs::TwistConstPtr& vel_msg ) {
            if( !is_calc_path_ ) return;
            if ( vel_msg->linear.x != 0.0 && vel_msg->angular.z != 0.0 ) {
                is_vel_stop_ = false;
                cnt_stop_ = 0;
                return;
            }
            cnt_stop_++;
            if ( cnt_stop_ > 10 ) is_vel_stop_ = true;
            return;
        }


    protected :
        PointCloud::Ptr cloud_obstacles_;
        geometry_msgs::Point robot_pt_;
        geometry_msgs::Twist robot_vel_;
        bool is_calc_path_;
        bool is_vel_stop_; 
        int cnt_stop_;
        geometry_msgs::Point transformPoint ( const geometry_msgs::Point& input_point, const std::string& org_frame, const std::string& tgt_frame ) {
            geometry_msgs::PointStamped pt_transformed;
            geometry_msgs::PointStamped pt;
            pt.header.frame_id = org_frame;
            pt.header.stamp = ros::Time(0);
            pt.point = input_point;
            if ( !tf_listener_.frameExists( tgt_frame ) ) {
                ROS_ERROR("tgt_frame is not Exists");
            } try {
                tf_listener_.transformPoint( tgt_frame, pt, pt_transformed );
            } catch ( tf::TransformException ex ) {
                ROS_ERROR( "%s",ex.what( ) );
            }
            return pt_transformed.point;
        }

    public :
        RobotSubscriber() : nh_(), pnh_("~") {
            cloud_obstacles_.reset ( new PointCloud() );
            sub_sensor_ = nh_.subscribe( "/sensor", 10, &RobotSubscriber::sensorCb, this );
            sub_vel_ = nh_.subscribe( "/cmd_vel_mux/input/teleop", 10, &RobotSubscriber::velocityCb, this );
            is_calc_path_ = false;
            cnt_stop_ = 0;
        }
};

class WayPointDWAActionServer : public RobotSubscriber {
    private :
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Publisher pub_dwa_calc_data_;
        actionlib::SimpleActionServer<dwa_tutorial::WayPointDWAAction> act_srv_;
        dwa_tutorial::WayPointDWAGoal goal_;

        void goalCb() {
            goal_ = *act_srv_.acceptNewGoal();
            ROS_INFO("\n[ WayPointDWAActionServer ] : Got a new goal...\n(x, y) = (%.3f[m], %.3f[m])\napproach_distance = %.3f\n", 
                goal_.waypoint.x, goal_.waypoint.y, goal_.dist_approach );
            controlCb();
        }
        void preemptCb() {
            ROS_INFO("\n[ WayPointDWAActionServer ] : Preempted\n" );
            dwa_tutorial::WayPointDWAResult result;
            result.result = false;
            act_srv_.setPreempted( result, "I got Preempted!" );
        }
        void controlCb() {
            if ( !act_srv_.isActive() || act_srv_.isPreemptRequested() ) return;
            ROS_INFO( "\n[ WayPointDWAActionServer ] : Start moving to waypoint...\n" );

            dwa::DWACalculationDataHandle dcdh;
            dwa_tutorial::WayPointDWAFeedback feedback;
            dwa_tutorial::WayPointDWAResult result;
            result.result = true;
            geometry_msgs::Point goal_pt_map = goal_.waypoint;
            double dist_approach = goal_.dist_approach;
            is_calc_path_ = true;
            do {
                if( is_vel_stop_ ) {
                    result.result = false;
                    break;
                }
                dcdh.initDWACalculationData();
                dcdh.addObstacle( cloud_obstacles_ );
                geometry_msgs::Point goal_pt_base = transformPoint( goal_pt_map, "map", "base_footprint" );
                dcdh.setTarget( goal_pt_base );
                pub_dwa_calc_data_.publish( dcdh.getDWACalculationData() );
                ros::spinOnce();
                feedback.dist_goal = std::hypotf( goal_pt_base.x, goal_pt_base.y );
                feedback.ang_goal = std::atan2( goal_pt_base.y, goal_pt_base.x );
                act_srv_.publishFeedback( feedback );
            } while ( dist_approach < feedback.dist_goal );
            ROS_INFO("\n[ WayPointDWAActionServer ] : Arrived at Waypoint\ndistance = %.3f\n", feedback.dist_goal );
            act_srv_.setSucceeded( result );
            is_calc_path_ = false;
            is_vel_stop_ = false;
            cnt_stop_ = 0;
        }

    public :
        WayPointDWAActionServer( std::string name ) : nh_(), pnh_("~"), act_srv_( nh_, name, false ) {
            pub_dwa_calc_data_ = nh_.advertise< dwa_tutorial::DWACalculationData >( "/dwa_calculation_data", 1 );
            act_srv_.registerGoalCallback( boost::bind( &WayPointDWAActionServer::goalCb, this ) );
            act_srv_.registerPreemptCallback ( boost::bind( &WayPointDWAActionServer::preemptCb, this ) );
            act_srv_.start();
            ROS_INFO( "%s Start the Server...\n", name.c_str() );
            int obs_size = pnh_.param<int>( "obstacle_size", 50 );
            cnt_stop_ = 0;
        }
};
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "dwa_waypoint_server");
    WayPointDWAActionServer wp_dwa_srv( ros::this_node::getName() );
    ros::spin();
}