#ifndef HEAT_CALCULATOR_H
#define HEAT_CALCULATOR_H

/// COMPONENT
#include <generic_obstacle_detection/point.h>

/// SYSTEM
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace od
{

enum class HeatCalculatorMethod
{
    NAIVE,
    PARTS,
    LINEAR,
    APPROX
};
enum class HeatType
{
    SLOPE,
    CURVATURE,
    SLOPE_DIFF,
    SBC15,
    PATSY,
    PATSY_OLD
};


template <class PointT>
class HeatCalculator
{
public:
    void calculate(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                   const std::vector<Point> &data,
                   std::vector<float>& heat);

private:
    void calculateHeatParts(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                            const std::vector<Point> &data,
                            std::vector<float>& heat);
    void calculateHeatComplete(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                               const std::vector<Point> &data,
                               std::vector<float>& heat);
    void calculateHeatLinear(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                               const std::vector<Point> &data,
                               std::vector<float>& heat);
    void calculateHeatApprox(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                               const std::vector<Point> &data,
                               std::vector<float>& heat);

public:
    HeatCalculatorMethod method_;
    HeatType heat_type_;


    std::size_t max_dist;

    double scale_h_;
    double scale_i_;
    double hzlimit_;
    double lookout_;
    double min_length_;

    double interval_min_;
    double interval_max_;

    double intensity_min_;
    double intensity_max_;

    int initial_offset_;
    int skip_;
    int step_;

private:
    void calculateHeat(const Point *currentP, const Point* nextP, const Point* prevP, float* heatP);

    void calculateHeatSlope(const Point *currentP, const Point* nextP, const Point* prevP, float* heatP);
    void calculateHeatSlopeDiff(const Point *currentP, const Point* nextP, const Point* prevP, float* heatP);
    void calculateHeatCurvature(const Point *currentP, const Point* nextP, const Point* prevP, float* heatP);
    void calculateHeatSBC15(const Point *currentP, const Point* nextP, const Point* prevP, float* heatP);
    void calculateHeatPATSY(const Point *currentP, const Point* nextP, const Point* prevP, float* heatP);
    void calculateHeatPATSYOld(const Point *currentP, const Point* nextP, const Point* prevP, float* heatP);
};

}

#endif // HEAT_CALCULATOR_H

