#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <dwa_tutorial/WayPointDWAAction.h>

#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

class WayPointDWAActionClient {
    private :
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Subscriber sub_tgt_pose_;
        actionlib::SimpleActionClient<dwa_tutorial::WayPointDWAAction> act_clt_;
        dwa_tutorial::WayPointDWAGoal goal_;

        void callbackTarget ( const geometry_msgs::PoseStampedConstPtr &msg ) {
            geometry_msgs::Point tgt_pt_map;
            tgt_pt_map.x = msg->pose.position.x;
            tgt_pt_map.y = msg->pose.position.y;
            goal_.waypoint = tgt_pt_map;
            sendGoal();
            ROS_INFO( "\n[ WayPointDWAActionClient ] : Set a new goal\n(x, y) = (%.3f, y = %.3f)\n", tgt_pt_map.x, tgt_pt_map.y );
            return;
        }
        void sendGoal() {
            act_clt_.sendGoal( goal_,
                                boost::bind( &WayPointDWAActionClient::doneCb, this, _1, _2 ),
                                boost::bind( &WayPointDWAActionClient::activeCb, this ),
                                boost::bind( &WayPointDWAActionClient::feedbackCb, this, _1 ) );
        }
        void doneCb( const actionlib::SimpleClientGoalState& state, const dwa_tutorial::WayPointDWAResultConstPtr& result ) {
            ROS_INFO( "\n[ WayPointDWAActionClient ] : Finished!!" );
            ROS_INFO( "\n[ WayPointDWAActionClient ] : Result : %s", ( result->result) ? "Succeeded" : "Failed" );
 
        }
        void activeCb( ) {
            ROS_INFO( "\n[ WayPointDWAActionClient ] : Goal just went active..." );
        }
        void  feedbackCb( const dwa_tutorial::WayPointDWAFeedbackConstPtr& feedback ) {
            ROS_INFO( "\n[ WayPointDWAActionClient ] : Got Feedback of Progress to Goal\ndistance = %.3f, angle = %.3f", feedback->dist_goal, feedback->ang_goal );
        }

    public : 
        WayPointDWAActionClient( std::string name ) : nh_(), pnh_("~"), act_clt_( "dwa_waypoint_server", true ) {
            ROS_INFO( "\n[ WayPointDWAActionClient ] : %s Waiting For Server...", name.c_str() );
            act_clt_.waitForServer();
            ROS_INFO("\n[ WayPointDWAActionClient ] : %s Got a Server...\nYou can set a new destination with ''2D Nav Goal( rviz top tab )''\n", name.c_str() );
            float approach_distance = pnh_.param<float>( "approach_distance", 0.3 );
            goal_.dist_approach = approach_distance;
            sub_tgt_pose_ = nh_.subscribe( "/move_base_simple/goal", 10, &WayPointDWAActionClient::callbackTarget, this );
        }




};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "dwa_waypoint_client");
    WayPointDWAActionClient wp_dwa_clt( ros::this_node::getName() );

    ros::spin();
}