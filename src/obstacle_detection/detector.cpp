/// HEADER
#include <generic_obstacle_detection/detector.h>

/// PROJECT
#include <generic_obstacle_detection/obstacle_classification.h>

using namespace csapex;
using namespace od;

template <class PointT>
void ObstacleDetector3D<PointT>::classify(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                          const std::vector<Point> &data,
                                          const std::vector<float>& heat,
                                          pcl::PointIndices& obstacles,
                                          pcl::PointIndices& floor)
{
    std::size_t rows = cloud->height;
    std::size_t cols = cloud->width;

    const auto threshold = heat_threshold_;

    auto* heatP = &heat[0];

    bool is_obstacle[1024];
    bool is_floor[1024];

    for(std::size_t col = 0; col < cols; ++col) {
        memset(&is_obstacle, 0, 1024 * sizeof(bool));
        memset(&is_floor, 0, 1024 * sizeof(bool));

        for(std::size_t row = 0; row < rows; ++row, ++heatP) {
            auto h = *heatP;

            if(h > threshold) {
                is_obstacle[row] = true;

            } else if(h > 0) {
                is_floor[row] = true;
            }
        }

        for(std::size_t row = 0; row < rows; ++row) {
            int index = row * cols + col;

            if(is_obstacle[row]) {
                obstacles.indices.push_back(index);

            } else if(is_floor[row]) {
                floor.indices.push_back(index);
            }
        }
    }
}

template <class PointT>
pcl::PointCloud<pcl::PointXYZRGB>::Ptr
ObstacleDetector3D<PointT>::generateHeatCloud(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                              const std::vector<float>& heat)
{
    const pcl::PointCloud<PointT>& in = *cloud;
    const std::vector<PointT, Eigen::aligned_allocator<PointT> >& points = in.points;

    std::size_t N = points.size();

    std::size_t rows = cloud->height;
    std::size_t cols = cloud->width;

    typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_heat(new pcl::PointCloud<pcl::PointXYZRGB>);
    output_heat->header = cloud->header;
    output_heat->width = cloud->width;
    output_heat->height = cloud->height;

    output_heat->points.resize(N);

    auto* heatP = &heat[0];
    for(std::size_t col = 0; col < cols; ++col) {
        auto* in_ptP = &cloud->points[col];
        auto* out_ptP = &output_heat->points[col];

        for(std::size_t row = 0; row < rows; ++row, ++heatP, in_ptP += cols, out_ptP += cols) {
            auto& p = *out_ptP;
            const PointT& dat = *in_ptP;

            p.x = dat.x;
            p.y = dat.y;
            p.z = dat.z;
            p.r = std::min(255, std::max(0, (int) (*heatP)));
        }
    }

    return output_heat;
}

template <class PointT>
pcl::PointCloud<pcl::PointXYZL>::Ptr
ObstacleDetector3D<PointT>::generateClassifiedCloud(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                                    const pcl::PointIndices& ground_indices,
                                                    const pcl::PointIndices& floor_indices)
{
    const pcl::PointCloud<PointT>& in = *cloud;
    const std::vector<PointT, Eigen::aligned_allocator<PointT> >& points = in.points;

    std::size_t N = points.size();

//    std::size_t rows = cloud->height;
//    std::size_t cols = cloud->width;

//    const auto threshold = heat_threshold_;

    typename pcl::PointCloud<pcl::PointXYZL>::Ptr output_classified_cloud(new pcl::PointCloud<pcl::PointXYZL>);
    output_classified_cloud->header = cloud->header;
    output_classified_cloud->width = cloud->width;
    output_classified_cloud->height = cloud->height;

    auto& class_points = output_classified_cloud->points;
    class_points.resize(N);

    {
        auto* orig_ptr = &points[0];
        auto* class_ptr = &class_points[0];
        for(std::size_t i = 0; i < N; ++i, ++orig_ptr, ++class_ptr) {
            const auto& pt_orig = *orig_ptr;
            auto& pt_class = *class_ptr;
            pt_class.x = pt_orig.x;
            pt_class.y = pt_orig.y;
            pt_class.z = pt_orig.z;
        }
    }

    for(int index : ground_indices.indices) {
        pcl::PointXYZL& pt = output_classified_cloud->points[index];
        pt.label =  (int) ObstacleClassification::UNSPECIFIED_OBSTACLE;
    }
    for(int index : floor_indices.indices) {
        pcl::PointXYZL& pt = output_classified_cloud->points[index];
        if( pt.label != (int) ObstacleClassification::UNSPECIFIED_OBSTACLE) {
            pt.label = (int) ObstacleClassification::FLOOR;
        }
    }

    return output_classified_cloud;
}

namespace od
{
template class ObstacleDetector3D<pcl::PointXYZ>;
template class ObstacleDetector3D<pcl::PointXYZI>;
template class ObstacleDetector3D<pcl::PointXYZRGB>;
}
