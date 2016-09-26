#ifndef DETECTOR_H
#define DETECTOR_H

/// COMPONENT
#include <generic_obstacle_detection/point.h>

/// SYSTEM
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

namespace od
{

template <class PointT>
class ObstacleDetector3D
{
public:
    void classify(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                  const std::vector<Point> &data,
                  const std::vector<float>& heat,
                  pcl::PointIndices& obstacles,
                  pcl::PointIndices& floor);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    generateHeatCloud(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                      const std::vector<float>& heat);

    pcl::PointCloud<pcl::PointXYZL>::Ptr
    generateClassifiedCloud(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                            const pcl::PointIndices& ground_indices,
                            const pcl::PointIndices& floor_indices);

public:
    double heat_threshold_;
};

}

#endif // DETECTOR_H
