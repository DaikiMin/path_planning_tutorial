#ifndef DWA_CALCULATION_DATA_HANDLE
#define DWA_CALCULATION_DATA_HANDLE

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <dwa_tutorial/DWACalculationData.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace dwa {
    class DWACalculationDataHandle {
        private :
            dwa_tutorial::DWACalculationDataPtr dwa_calc_data_;
        public :
            DWACalculationDataHandle ( );
            void initDWACalculationData (  );
            void setTarget( const geometry_msgs::Point target  );
            void addObstacle ( const PointCloud::Ptr obstacle_cloud );
            dwa_tutorial::DWACalculationData getDWACalculationData ( );
    };

}

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;


#endif