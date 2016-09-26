/// HEADER
#include <generic_obstacle_detection/low_pass_filter.h>

using namespace od;

template <typename PointT>
void LowPassFilter<PointT>::lowPass(const typename pcl::PointCloud<PointT>::ConstPtr &cloud, std::vector<Point> &data)
{
    const double max_distance = max_distance_;
    const double zlimit = zlimit_;
    const double alpha = alpha_;

    std::size_t rows = cloud->height;
    std::size_t cols = cloud->width;

    auto* dataP = &data[0];
    for(std::size_t col = 0; col < cols; ++col) {
        for(std::size_t row = 0; row < rows; ++row, ++dataP) {
            auto& current = *dataP;

            Point* prev = nullptr;
            auto* prevP = dataP-1;
            for(int r = row - 1; r >= 0; --r, --prevP) {
                const auto& p = *prevP;
                if(std::isnan(p.x)) {
                    continue;
                    //                    } else if(std::hypot(p.x - current.x, p.y - current.y) > max_distance) {
                } else if(std::abs(p.x - current.x) > max_distance) {
                    continue;
                } else {
                    prev = prevP;
                    break;
                }
            }


            if(!prev) {
                current.z = std::numeric_limits<double>::quiet_NaN();
            } else if(std::abs(prev->z - current.z) < zlimit) {
                current.z = prev->z + alpha * (current.z - prev->z);
            }
        }
    }
}

namespace od
{
template class LowPassFilter<pcl::PointXYZ>;
template class LowPassFilter<pcl::PointXYZI>;
template class LowPassFilter<pcl::PointXYZRGB>;
}
