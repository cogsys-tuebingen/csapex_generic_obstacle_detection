/// COMPONENT
#include <generic_obstacle_detection/point.h>

/// SYSTEM
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace od
{

template <class PointT>
class LowPassFilter
{
public:
    void lowPass(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                 std::vector<Point> &data);

public:
    double alpha_;
    double zlimit_;
    double max_distance_;
};

}
