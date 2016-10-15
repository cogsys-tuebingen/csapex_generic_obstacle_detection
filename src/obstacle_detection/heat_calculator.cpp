/// HEADER
#include <generic_obstacle_detection/heat_calculator.h>

/// PROJECT
#include <csapex/utility/assert.h>

using namespace od;

template <class PointT>
void HeatCalculator<PointT>::calculate(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                       const std::vector<Point> &data,
                                       std::vector<float>& heat)
{
    switch(method_) {
    case HeatCalculatorMethod::NAIVE:
        return calculateHeatComplete(cloud, data, heat);
    case HeatCalculatorMethod::PARTS:
        return calculateHeatParts(cloud, data, heat);
    case HeatCalculatorMethod::LINEAR:
        return calculateHeatLinear(cloud, data, heat);
    case HeatCalculatorMethod::APPROX:
        return calculateHeatApprox(cloud, data, heat);
    default:
        throw std::runtime_error("method not implemented");
    }

}

template <class PointT>
void HeatCalculator<PointT>::calculateHeatParts(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                                const std::vector<Point> &data,
                                                std::vector<float>& heat)
{
    std::size_t rows = cloud->height;
    std::size_t cols = cloud->width;

    const Point* datP = &data[0];
    float* heatP = &heat[0];

    const auto skip = skip_;
    const auto step = step_;
    const auto initial_offset = initial_offset_;
    const auto hzlimit = hzlimit_;
    const auto lookout = lookout_;

    for(std::size_t col = 0; col < cols; ++col) {

        std::vector<const Point*> prev(rows, nullptr);
        std::vector<const Point*> next(rows, nullptr);

        auto* datP_start = datP;
        auto* heatP_start = heatP;

        for(std::size_t row = 0; row < rows; row += skip, heatP += skip, datP += skip) {
            const Point& current = *datP;

            const Point* prevP_tmp = datP - initial_offset;
            const Point* prevP = nullptr;

            auto x_ref = current.x;
            auto z_ref = current.z;

            for(int r = row-initial_offset, end = std::max(0, (int) (row-max_dist)); r >= end; r -= step, prevP_tmp -= step) {
                const Point& previous = *prevP_tmp;
                if(std::isnan(previous.z) || std::abs(previous.z - z_ref) > hzlimit) {
                    continue;
                }

                if(std::abs(previous.x - x_ref) >= lookout) {
                    break;
                }

                prevP = prevP_tmp;
            }

            prev[row] = prevP;
        }

        datP = datP_start;
        heatP = heatP_start;

        for(std::size_t row = 0; row < rows; row += skip, heatP += skip, datP += skip) {
            const Point& current = *datP;

            const Point* nextP_tmp = datP + initial_offset;
            const Point* nextP = nullptr;

            auto x_ref = current.x;
            auto z_ref = current.z;

            for(std::size_t r = row+initial_offset, end = std::min(rows, row+max_dist); r < end; r += step, nextP_tmp += step) {
                const Point& next = *nextP_tmp;
                if(std::isnan(next.z) || std::abs(next.z - z_ref) > hzlimit) {
                    continue;
                }

                if(std::abs(next.x - x_ref) >= lookout) {
                    break;
                }

                nextP = nextP_tmp;
            }

            next[row] = nextP;
        }

        datP = datP_start;
        heatP = heatP_start;

        for(std::size_t row = 0; row < rows; row += skip, heatP += skip, datP += skip) {
            auto* prevP = prev[row];
            auto* nextP = next[row];

            calculateHeat(datP, nextP, prevP, heatP);
        }
    }
}

template <class PointT>
void HeatCalculator<PointT>::calculateHeatComplete(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                                   const std::vector<Point> &data, std::vector<float>& heat)
{
    std::size_t rows = cloud->height;
    std::size_t cols = cloud->width;

    apex_assert(rows * cols == data.size());
    apex_assert(rows * cols == cloud->points.size());

    const Point* datP = &data[0];
    float* heatP = &heat[0];

    const auto skip = skip_;
    const auto step = step_;
    const auto initial_offset = initial_offset_;
    const auto hzlimit = hzlimit_;
    const auto lookout = lookout_;

    for(std::size_t col = 0; col < cols; ++col) {
        datP = &data[col * rows];
        for(std::size_t row = 0; row < rows; row += skip, heatP += skip, datP += skip) {
            const Point& current = *datP;

            const Point* prevP_tmp = datP - initial_offset;
            const Point* prevP = nullptr;

            auto x_ref = current.x;
            auto z_ref = current.z;

            for(int r = row-initial_offset, end = std::max(0, (int) (row-max_dist)); r >= end; r -= step, prevP_tmp -= step) {
                const Point& previous = *prevP_tmp;
                if(std::isnan(previous.z) || std::abs(previous.z - z_ref) > hzlimit) {
                    continue;
                }

                if(std::abs(previous.x - x_ref) >= lookout) {
                    break;
                }

                prevP = prevP_tmp;
            }

            const Point* nextP_tmp = datP + initial_offset;
            const Point* nextP = nullptr;

            for(std::size_t r = row+initial_offset, end = std::min(rows, row+max_dist); r < end; r += step, nextP_tmp += step) {
                const Point& next = *nextP_tmp;
                if(std::isnan(next.z) || std::abs(next.z - z_ref) > hzlimit) {
                    continue;
                }

                if(std::abs(next.x - x_ref) >= lookout) {
                    break;
                }

                nextP = nextP_tmp;
            }

            calculateHeat(datP, nextP, prevP, heatP);
        }
    }
}

template <class PointT>
void HeatCalculator<PointT>::calculateHeatLinear(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                                 const std::vector<Point> &data, std::vector<float>& heat)
{
    std::size_t rows = cloud->height;
    std::size_t cols = cloud->width;

    const Point* datP = &data[0];
    float* heatP = &heat[0];

    const auto skip = skip_;
    const auto step = step_;
    const auto initial_offset = initial_offset_;
    const auto hzlimit = hzlimit_;
    const auto lookout = lookout_;


    for(std::size_t col = 0; col < cols; ++col) {

        std::vector<const Point*> prev(rows, nullptr);
        std::vector<const Point*> next(rows, nullptr);

        auto* datP_start = datP;
        auto* heatP_start = heatP;

        for(std::size_t row = 0; row < rows; row += skip, heatP += skip, datP += skip) {
            const Point* currentP = datP;
            const Point& current = *currentP;

            const Point* nextP_tmp = datP + initial_offset;
            const Point* nextP = nullptr;

            std::size_t nextP_row = row;

            auto x_ref = current.x;
            auto z_ref = current.z;

            for(std::size_t r = row+initial_offset, end = std::min(rows, row+max_dist); r < end; r += step, nextP_tmp += step) {
                const Point& next = *nextP_tmp;
                if(std::isnan(next.z) || std::abs(next.z - z_ref) > hzlimit) {
                    continue;
                }

                if(std::abs(next.x - x_ref) >= lookout) {
                    break;
                }

                nextP = nextP_tmp;
                nextP_row = r;

                auto current_distance = std::abs(nextP->x - current.x);
                if(current_distance <= lookout) {
                    const Point* previous_best = prev[nextP_row];
                    if(previous_best == nullptr) {
                        prev[nextP_row] = currentP;
                    } else {
                        auto last_distance = std::abs(nextP->x - previous_best->x);
                        if(current_distance > last_distance) {
                            prev[nextP_row] = currentP;
                        }
                    }
                }
            }

            if(row != nextP_row) {
                next[row] = nextP;
            }
        }

        datP = datP_start;
        heatP = heatP_start;

        for(std::size_t row = 0; row < rows; row += skip, heatP += skip, datP += skip) {
            auto* prevP = prev[row];
            auto* nextP = next[row];

            calculateHeat(datP, nextP, prevP, heatP);
        }
    }
}


template <class PointT>
void HeatCalculator<PointT>::calculateHeatApprox(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                                 const std::vector<Point> &data, std::vector<float>& heat)
{
    std::size_t rows = cloud->height;
    std::size_t cols = cloud->width;

    const Point* datP = &data[0];
    float* heatP = &heat[0];

    const auto skip = skip_;
    const auto initial_offset = initial_offset_;

    for(std::size_t col = 0; col < cols; ++col) {
        for(std::size_t row = 0; row < rows; row += skip, heatP += skip, datP += skip) {
            const Point* prevP = datP - std::min((int) row, initial_offset);
            const Point* nextP = datP + std::min(((int) rows) - 1 - ((int) row), initial_offset);

            calculateHeat(datP, nextP, prevP, heatP);
        }
    }
}


template <class PointT>
void HeatCalculator<PointT>::calculateHeat(const Point* currentP, const Point* nextP, const Point* prevP, float* heatP)
{
    switch(heat_type_) {
    default:
    case HeatType::SLOPE:
        calculateHeatSlope(currentP, nextP, prevP, heatP);
        break;
    case HeatType::SLOPE_DIFF:
        calculateHeatSlopeDiff(currentP, nextP, prevP, heatP);
        break;
    case HeatType::CURVATURE:
        calculateHeatCurvature(currentP, nextP, prevP, heatP);
        break;
    case HeatType::SBC15:
        calculateHeatSBC15(currentP, nextP, prevP, heatP);
        break;
    case HeatType::IROS16:
        calculateHeatPATSY(currentP, nextP, prevP, heatP);
        break;
    case HeatType::PATSY_OLD:
        calculateHeatPATSYOld(currentP, nextP, prevP, heatP);
        break;

        break;
    }
}

template <class PointT>
void HeatCalculator<PointT>::calculateHeatSlope(const Point* currentP, const Point* nextP, const Point* prevP, float* heatP)
{
    if(prevP && nextP) {
        auto dx = std::abs(prevP->x - nextP->x);
        auto dz = std::abs(prevP->z - nextP->z);

        auto len = dx;//std::hypot(prevP->x - nextP->x, prevP->y - nextP->y);

        if(len < min_length_) {
            if(dz > 0.1) {
                *heatP = std::abs(scale_h_ * 1.0);
            }

        } else {
            if(dz > 0.01) {
                *heatP = std::abs(scale_h_ * dz / dx);
            } else {
                // very likely no obstacle -> still heat == 0 is bad
                *heatP = scale_h_ * 0.01;
            }
        }

    } else {
        if(!prevP) {
            //            *heatP = std::abs(scale_h_ * 1.0);
        }
    }
}

template <class PointT>
void HeatCalculator<PointT>::calculateHeatSlopeDiff(const Point* currentP, const Point* nextP, const Point* prevP, float* heatP)
{
    if(prevP && nextP) {
        auto dx = std::abs(prevP->x - nextP->x);
        auto dz = std::abs(prevP->z - nextP->z);

        auto len = dx;//std::hypot(prevP->x - nextP->x, prevP->y - nextP->y);

        if(len < min_length_) {
            if(dz > 0.1) {
                *heatP = std::abs(scale_h_ * 1.0);
            }

        } else {
            if(dz > 0.01) {
                auto dP = std::atan((prevP->z - currentP->z) / (prevP->x - currentP->x)) / M_PI;
                auto dN = std::atan((currentP->z - nextP->z) / (currentP->x - nextP->x)) / M_PI;
                *heatP = std::abs(scale_h_ * std::abs(dP - dN));
            } else {
                // very likely no obstacle -> still heat == 0 is bad
                *heatP = scale_h_ * 0.01;
            }
        }

    } else {
        if(!prevP) {
            //            *heatP = std::abs(scale_h_ * 1.0);
        }
    }
}

template <class PointT>
void HeatCalculator<PointT>::calculateHeatCurvature(const Point* currentP, const Point* nextP, const Point* prevP, float* heatP)
{    if(prevP && nextP) {
        auto dx = std::abs(prevP->x - nextP->x);
        auto dz = std::abs(prevP->z - nextP->z);

        auto len = dx;//std::hypot(prevP->x - nextP->x, prevP->y - nextP->y);

        if(len < min_length_) {
            if(dz > 0.1) {
                *heatP = std::abs(scale_h_ * 1.0);
            }

        } else {
            if(dz > 0.01) {
                Point Center;
                double yDelta_a= currentP->y - prevP->y;
                double xDelta_a= currentP->x - prevP->x;
                double yDelta_b= nextP->y - currentP->y;
                double xDelta_b= nextP->x - currentP->x;

                double dRadius;

                if (fabs(xDelta_a) <= 0.000000001 && fabs(yDelta_b) <= 0.000000001){
                    Center.x= 0.5*(currentP->x + nextP->x);
                    Center.y= 0.5*(prevP->y + currentP->y);
                    Center.z= prevP->z;
                    dRadius= std::hypot(Center.x - currentP->x, Center.y - currentP->y);
                }

                // IsPerpendicular assure that xDelta(s) are not zero
                double aSlope=yDelta_a/xDelta_a; //
                double bSlope=yDelta_b/xDelta_b;
                if (fabs(aSlope-bSlope) <= 0.000000001){	// checking whether the given points are colinear.
                    *heatP = scale_h_ * 0.01;
                    return ;
                }

                // calc center
                Center.x= (aSlope*bSlope*(prevP->y - prevP->y) + bSlope*(prevP->x + currentP ->x)
                           - aSlope*(currentP->x+prevP->x) )/(2* (bSlope-aSlope) );
                Center.y = -1*(Center.x - (prevP->x+currentP->x)/2)/aSlope +  (prevP->y+currentP->y)/2;
                Center.z= prevP->z;

                dRadius= std::hypot(Center.x - currentP->x, Center.y - currentP->y);

                double k = 1.0 / dRadius;

                *heatP = std::abs(scale_h_ * k * 1e-3);
            } else {
                // very likely no obstacle -> still heat == 0 is bad
                *heatP = scale_h_ * 0.01;
            }
        }

    } else {
        if(!prevP) {
            //            *heatP = std::abs(scale_h_ * 1.0);
        }
    }
}

template <class PointT>
void HeatCalculator<PointT>::calculateHeatSBC15(const Point* currentP, const Point* nextP, const Point* prevP, float* heatP)
{
    if(prevP && nextP) {
        auto dx = std::abs(prevP->x - nextP->x);
        auto len = dx;//std::hypot(prevP->x - nextP->x, prevP->y - nextP->y);

        if(len < min_length_) {
            auto dz = std::abs(prevP->z - nextP->z);
            if(dz > 0.1) {
                *heatP = std::abs(scale_h_ * 1.0);
            }

        } else {
            //            auto delta = dz / len;
            //            *heatP = std::abs(scale_h_ * delta);

            auto dz = std::abs(prevP->z - nextP->z);
            if(dz > 0.01) {
                auto mean_z = 0.5f * (prevP->z + nextP->z);
                if(currentP->z >= mean_z) {
                    *heatP = std::abs(scale_h_ * std::atan(dz / dx) / M_PI);
                } else {
                    auto dz_prev = std::abs(prevP->z - currentP->z);
                    auto dz_next = std::abs(currentP->z - nextP->z);
                    auto delta = std::min(dz_prev, dz_next);
                    *heatP = std::abs(scale_h_ * delta);
                }
            } else {
                // very likely no obstacle -> still heat == 0 is bad
                *heatP = scale_h_ * 0.01;
            }
        }

    } else {
        if(!prevP) {
            //            *heatP = std::abs(scale_h_ * 1.0);
        }
    }
}

template <class PointT>
void HeatCalculator<PointT>::calculateHeatPATSY(const Point* currentP, const Point* nextP, const Point* prevP, float* heatP)
{
    float heat_depth = 0.0;

    // > 0 for floor!

    if(currentP->z > interval_max_) {
        heat_depth = 1.0;
    } else if(currentP->z < interval_min_) {
        heat_depth = 1.0;

    } else if(prevP && nextP) {
        auto dx = std::abs(prevP->x - nextP->x);
        auto len = dx;//std::hypot(prevP->x - nextP->x, prevP->y - nextP->y);


        if(len < min_length_) {
            auto dz = std::abs(prevP->z - nextP->z);
            if(dz > 0.1) {
                heat_depth = 1.0;
            }

        } else {
            //            auto delta = dz / len;
            //            *heatP = std::abs(scale_h_ * delta);

            auto dz = std::abs(prevP->z - nextP->z);
            if(dz > 0.01) {
                auto mean_z = 0.5f * (prevP->z + nextP->z);
                if(currentP->z >= mean_z) {
                    //                    heat_depth = std::abs(std::atan(dz / dx) / M_PI);
                    auto dP = std::atan((prevP->z - currentP->z) / (prevP->x - currentP->x)) / M_PI;
                    auto dN = std::atan((currentP->z - nextP->z) / (currentP->x - nextP->x)) / M_PI;
                    heat_depth = std::abs(dP - dN);
                } else {
                    auto dz_prev = std::abs(prevP->z - currentP->z);
                    auto dz_next = std::abs(currentP->z - nextP->z);
                    auto delta = std::min(dz_prev, dz_next);
                    heat_depth = std::abs(delta);
                }
            } else {
                heat_depth = 0.01;
            }
        }


    } else {
        if(prevP || nextP) {
            //            *heatP = std::abs(scale_h_ * 1.0);
            //            heat_depth = 0.01;
        }
    }


    double heat_intens = 0.0;

    if(prevP && nextP) {
        // we need to model the distance dependency of the intensity!
        double icurr = currentP->intensity;
        double inext = nextP->intensity;
        double iprev = prevP->intensity;

//        auto dist = [](const Point& p) {
//            return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
//        };
//        double distcurrent = dist(*currentP);
//        double distnext = dist(*nextP);
//        double distprev = dist(*prevP);

        /**
         * we assume that the sensor is only tilted, not rolled
         * further we assume that the floor is flat and the normal is
         *   N = (0,0,1)^T
         *
         * calculate lambertian reflection model
         *   R = dot(L, N)
         * where L is the direction of the light ray with angle alpha:
         *   alpha = alpha' + tilt_angle
         * and
         *   alpha' = atan2(pt.y, pt.x)
         * ->
         *   L = / cos(-alpha')   0  sin(-alpha') \
         *       |     0          1          0     |  .  (1, 0, 0)^T
         *       \ -sin(-alpha')  0  cos(-alpha')  /
         *
         *     = (cos(alpha'), 0, -sin(alpha'))^T
         *
         * ->  R = -sin(alpha')
         */
//        auto reflectance = [this](const Point& pt) {
//            //        double tilt_angle = angle_;
//            double alpha = std::atan2(pt.z, pt.x);
//            double R = -std::sin(alpha);
//            return reflectance_ * R;
//        };

//        double rcurr = reflectance(*currentP);
//        double rnext = reflectance(*nextP);
//        double rprev = reflectance(*prevP);

        /**
         * Let I be the intensity of light emitted.
         * I' is measured at point p and falls of quadratically:
         *   I' = 1/dist(p)² * I
         *
         * The intensity I' is partially reflected back to the sensor.
         * Let R be the reflectance of the material at point p, then
         *   I'r = R * I'
         * is the reflected intensity.
         * For I'r the quadratic distance relation holds as well, so
         *   Ir = 1/dist(p)² * I'r
         * is received at the sensor.
         *
         * Resulting formula for the received light:
         *   Ir = 1/dist(p)² * R * 1 / dist(p)² * I
         *      = 1/dist(p)⁴ * R * I
         *
         *   -> I = Ir * dist(p)⁴ / R
         *
         *   Ip = 1/dist(prev)⁴ * Rp * I
         *      = 1/dist(prev)⁴ * Ir * dist(p)⁴
         *      = dist(p)⁴/dist(prev)⁴ * Ir
         *   In = dist(p)⁴/dist(next)⁴ * Ir
         */

//        double I = icurr * std::pow(distcurrent, power_) / rcurr;
//        double Ip = 1.0 / std::pow(distprev, power_) * rprev * I;
//        double In = 1.0 / std::pow(distnext, power_) * rnext * I;

////        *heatP = scale_i_ * std::abs(icurr - I * std::pow(distcurrent, 2));

//        double diprev = std::abs(Ip - iprev);
//        double dinext = std::abs(inext - In);


//        double dcurr_sqr = std::pow(dist(*currentP), power_);
//        double dnext_sqr = std::pow(dist(*nextP), power_);
//        double dprev_sqr = std::pow(dist(*prevP), power_);

//        double inext_exp = dcurr_sqr / dnext_sqr * icurr;
//        double iprev_exp = dcurr_sqr / dprev_sqr * icurr;

//        double diprev = std::abs(iprev_exp - iprev);
//        double dinext = std::abs(inext - inext_exp);


        //        double dcurr = currentP->x;
        //        double dnext = nextP->x;
        //        double dprev = prevP->x;

        //        double dcurr_sqr = dcurr * dcurr;
        //        double dnext_sqr = dprev * dnext;
        //        double dprev_sqr = dprev * dprev;

        //        double inext_exp = dcurr_sqr / dnext_sqr * icurr;
        //        double iprev_exp = dcurr_sqr / dprev_sqr * icurr;

        //        if(icurr > inext || icurr > iprev) {


        double icurr_exp = (icurr);// * std::pow((distcurrent + distance_offset_), power_);
        double inext_exp = (inext);// * std::pow((distnext + distance_offset_), power_);
        double iprev_exp = (iprev);// * std::pow((distprev + distance_offset_), power_);

//        double icurr_exp = (icurr);// / std::pow(distcurrent * reflectance_, power_);
//        double inext_exp = (inext);// / std::pow(distnext * reflectance_, power_);
//        double iprev_exp = (iprev);// / std::pow(distprev * reflectance_, power_);


        double diprev = std::abs(icurr_exp - iprev_exp);
        double dinext = std::abs(inext_exp - icurr_exp);

        double deltai = std::max(diprev, dinext);
        if(icurr > intensity_max_) {
            heat_intens = 1.0;
        } else if(icurr >= intensity_min_) {
            heat_intens = deltai / 2048.0;//(deltai - intensity_min_) / (intensity_max_ - intensity_min_);
        }
        //        }
    }

    *heatP = std::abs(scale_h_ * heat_depth + scale_i_ * heat_intens);
}

template <class PointT>
void HeatCalculator<PointT>::calculateHeatPATSYOld(const Point* currentP, const Point* nextP, const Point* prevP, float* heatP)
{

    double h_height = 0.0;

    if(prevP && nextP) {
        double hprev = currentP->z - prevP->z;
        double hnext = nextP->z - currentP->z;
        h_height = std::abs(scale_h_ * (hprev - hnext));
    }

    if(h_height != h_height) {
        h_height = 0.0;
    }

    double h_intens = 0.0;

    if(prevP && nextP) {
        double iprev = (currentP->intensity - prevP->intensity) / 1024.;
        double inext = (nextP->intensity - currentP->intensity) / 1024.;
        h_intens = std::abs(scale_i_ * (iprev - inext));
    }

    if(h_intens != h_intens) {
        h_intens = 0.0;
    }

    double h = h_height + h_intens;
    *heatP = h;
}

namespace od
{
template class HeatCalculator<pcl::PointXYZ>;
template class HeatCalculator<pcl::PointXYZI>;
template class HeatCalculator<pcl::PointXYZRGB>;
}
