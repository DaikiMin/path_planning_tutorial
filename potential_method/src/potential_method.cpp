#include <potential_method/potential_method.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace Potential;

void PotentialMethod::dispPotentialField( const std::vector<PotentialField>& pot_field ) {
    visualization_msgs::MarkerArray pot_mrk;
    int marker_id = 0;
    double r0 = 1.0, g0 = 0.0, b0 = 0.0;
    double rh = 0.0, gh = 0.5, bh = 1.0;
    double h = 2.0;
    for ( auto& pf : pot_field ) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "potential_field";
        marker.id = marker_id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = pf.x;
        marker.pose.position.y = pf.y;
        marker.pose.position.z = pf.potential;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.1;

        double pot = pf.org_potential + 1.0;
        marker.color.r = r0 + ( rh - r0 ) * pot/ h;
        marker.color.g = g0 + ( gh - g0 ) * pot/ h;
        marker.color.b = b0 + ( bh - b0 ) * pot/ h;
        marker.color.a = 0.5f;

        marker.lifetime = ros::Duration(0.3);
        marker_id++;
        pot_mrk.markers.push_back( marker );
    }
    pub_field_.publish( pot_mrk );
    return;
}

bool PotentialMethod::calPotentialField( 
    const geometry_msgs::Point& goal, 
    const PointCloud::Ptr obstacles, 
    std::vector<PotentialField>* pot_field ) 
{
    if( obstacles->points.size() == 0 ) return false;

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    double radius = 0.5;
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud (obstacles);
    pcl::PointXYZ search_pt;

    double weight_obs = weight_obs_;
    double weight_goal = weight_goal_;
    double grid_width = grid_width_;
    std::vector<PotentialField> pf;

    for ( double x = -5.0; x < 5.0; x += grid_width ) {
        for ( double y = -5.0; y < 5.0; y += grid_width ) {
            // Searching for obstacles : ;
            double obs_pot = 0.0, goal_pot = 0.0;
            PotentialField tmp;
            geometry_msgs::Point obs_pt;
            search_pt.x = x;search_pt.y = y;search_pt.z = 0.0;
             if ( kdtree.radiusSearch (search_pt, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance, 1) > 0 ) {
                obs_pt.x = obstacles->points[ pointIdxRadiusSearch[0] ].x; 
                obs_pt.y = obstacles->points[ pointIdxRadiusSearch[0] ].y; 
                // Calculating Obstacle Potential :
                if ( std::hypotf( x - obs_pt.x, y - obs_pt.y ) < 0.05 ) obs_pot = 1.0;
                else obs_pot = calPotential( x, y, obs_pt, false );
            } 
            // Calculating Goal Potential :
            if( std::hypotf( x - goal.x, y -goal.y ) < 0.05 ) goal_pot = -1.0;
            else goal_pot = calPotential( x, y, goal, true );
            tmp.potential = ( weight_obs * obs_pot ) + ( weight_goal * goal_pot );
            tmp.org_potential = obs_pot + goal_pot;
            tmp.x = x;
            tmp.y = y;
            pf.push_back( tmp );
        }
    }
    *pot_field = pf;
    return true;
}

bool PotentialMethod::calPath( 
    const geometry_msgs::Point& goal, 
    const PointCloud::Ptr obstacles, 
    const geometry_msgs::Twist& curt_vel, 
    geometry_msgs::Twist* path ) 
{
    if( obstacles->points.size() == 0 ) return false;
    geometry_msgs::Twist tmp_path;

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    double radius = 0.5;
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud (obstacles);
    pcl::PointXYZ search_pt;

    double weight_obs = weight_obs_;
    double weight_goal = weight_goal_;
    double delta = delta_;
    double goal_dist = std::hypotf( goal.x, goal.y );

    geometry_msgs::Point obs_pt;
    search_pt.x = 0.0;search_pt.y = 0.0;search_pt.z = 0.0;
    double obs_pot = 0.0, obs_pot_x_del = 0.0, obs_pot_y_del = 0.0;
    double goal_pot = 0.0, goal_pot_x_del = 0.0, goal_pot_y_del = 0.0;

    
    // Calculating Obstacle Potential :
    if ( kdtree.radiusSearch (search_pt, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance, 1) > 0 ) {
        obs_pt.x = obstacles->points[ pointIdxRadiusSearch[0] ].x; 
        obs_pt.y = obstacles->points[ pointIdxRadiusSearch[0] ].y;
        if ( std::hypotf( obs_pt.x, obs_pt.y ) < 0.05 ) {
            obs_pot = 1.0;
            obs_pot_x_del = 1.0;
            obs_pot_y_del = 1.0;
        } else {
            obs_pot = calPotential( 0.0, 0.0, obs_pt, false );
            obs_pot_x_del = calPotential(delta, 0.0, obs_pt, false );
            obs_pot_y_del = calPotential( 0.0, delta, obs_pt, false );
        }
    }
    // Calculating Goal Potential :
    if(goal_dist < 0.05 ) {
        goal_pot = -1.0;
        goal_pot_x_del = -1.0; 
        goal_pot_y_del = -1.0;
    } else {
        goal_pot = calPotential( 0.0, 0.0, goal, true );
        goal_pot_x_del = calPotential( delta, 0.0, goal, true );
        goal_pot_y_del = calPotential( 0.0, delta, goal, true );
    }
    double pot_xy = ( weight_obs * obs_pot ) + ( weight_goal * goal_pot ); 
    double pot_x_del = ( weight_obs * obs_pot_x_del ) + ( weight_goal * goal_pot_x_del ); 
    double pot_y_del = ( weight_obs * obs_pot_y_del ) + ( weight_goal * goal_pot_y_del );
    // #polarization of the potential field ( Calculate the gradient of the potential field ) :
    double vx = -( pot_x_del - pot_xy ) / delta;
    double vy = -( pot_y_del - pot_xy ) / delta;
    double v = std::hypotf( vx, vy );
    double vel =  1.3 / ( 1 + std::exp( -1.3 * 1.3 * ( goal_dist - 1 ) ) ); // Logistic function

    vx /= v / vel;
    vy /= v / vel;
    
    geometry_msgs::Point next_pt;
    next_pt.x = vx;
    next_pt.y = vy;
    tmp_path.linear.x = std::hypotf( next_pt.x, next_pt.y );
    tmp_path.angular.z = std::atan2( next_pt.y, next_pt.x );
    *path = tmp_path;
    return true;
}

PotentialMethod::PotentialMethod() : nh_() {
    pub_field_ = nh_.advertise< visualization_msgs::MarkerArray >( "/potential_field", 1 );
    setWeightObstacle( 1.0 );
    setWeightGoal( 1.0 );
    setDelta( 0.1 );
    setGridWidth( 0.25 );
}

bool PotentialMethod::potentialField( const geometry_msgs::Point& goal,const PointCloud::Ptr obstacles ) {
    std::vector<PotentialField> pot_field;
    calPotentialField( goal, obstacles, &pot_field );
    dispPotentialField( pot_field );
    return true;
}

bool PotentialMethod::generatePath( 
    const geometry_msgs::Point& goal,
    const PointCloud::Ptr obstacles, 
    const geometry_msgs::Twist& curt_vel, 
    geometry_msgs::Twist* path ) 
{   
    geometry_msgs::Twist tmp_path;
    calPath( goal, obstacles, curt_vel, &tmp_path );
    *path = tmp_path;
    return true;
}


