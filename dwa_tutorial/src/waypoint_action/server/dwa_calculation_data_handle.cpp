#include <dwa_tutorial/dwa_calculation_data_handle.h>

namespace dwa {
        DWACalculationDataHandle::DWACalculationDataHandle ( ) { 
            dwa_calc_data_.reset ( new dwa_tutorial::DWACalculationData );
            initDWACalculationData ( ); 
        }
        void DWACalculationDataHandle::initDWACalculationData (  ) { 
            dwa_tutorial::DWACalculationData init_data;
            *dwa_calc_data_ = init_data;
        }
        void DWACalculationDataHandle::setTarget( const geometry_msgs::Point target  ) { 
            dwa_calc_data_->target = target; 
            dwa_calc_data_->target_distance = std::hypotf( target.x, target.y);
			dwa_calc_data_->target_angle = std::atan2 ( target.y, target.x );
        }
        void DWACalculationDataHandle::addObstacle ( const PointCloud::Ptr obstacle_cloud ) {
            dwa_tutorial::DWACalculationData tmp_fp = *dwa_calc_data_;
            for ( auto& pt : obstacle_cloud->points ) {
                dwa_tutorial::Obstacle obstacle;
                obstacle.center.x = pt.x;
                obstacle.center.y = pt.y;
                geometry_msgs::Point tmp_pt;
                tmp_pt.x = pt.x;
                tmp_pt.y = pt.y;
                obstacle.points.push_back( tmp_pt ); 
                tmp_fp.obstacles.push_back( obstacle );
            }
            *dwa_calc_data_ = tmp_fp;
        }
        dwa_tutorial::DWACalculationData DWACalculationDataHandle::getDWACalculationData ( ) { return *dwa_calc_data_; }
}