/// HEADER
#include "point_histogram.h"

/// SYSTEM
#include <limits>

using namespace csapex;

namespace od {
template <class PointT>
bool sortPts(const std::pair<PointT, unsigned>& a, const std::pair<PointT, unsigned>& b) {
    return a.first.z < b.first.z;
}
}

template <class PointT>
PointHistogram<PointT>::PointHistogram(const Histogram::Params &params)
    : Histogram(params), points_(params.w * params.h)
{
    reset();
}

template <class PointT>
void PointHistogram<PointT>::reset()
{
    minx = std::numeric_limits<unsigned>::max();
    maxx = std::numeric_limits<unsigned>::min();
    miny = std::numeric_limits<unsigned>::max();
    maxy = std::numeric_limits<unsigned>::min();
}

template <class PointT>
void PointHistogram<PointT>::insert(const PointT &pt, unsigned int idx)
{
    int x = mapX(pt.x);
    int y = mapY(pt.y);

    if(!inside(x, y)) {
        return;
    }

    unsigned ux = x;
    unsigned uy = y;


    if(ux < minx) minx = ux;
    if(ux > maxx) maxx = ux;
    if(uy < miny) miny = uy;
    if(uy > maxy) maxy = uy;

    points_[index(ux, uy)].push_back(std::make_pair(pt, idx));
}

template <class PointT>
void PointHistogram<PointT>::sort()
{
    std::size_t padding = 4;
    for(unsigned i = 0, m = points_.size(); i < m; ++i) {
        std::vector<std::pair<PointT, unsigned int> >& stack = points_[i];
        std::sort(stack.begin(), stack.end(), sortPts<PointT>);
        if(stack.size() > 3 * padding) {
            stack.erase(stack.end() - padding, stack.end());
            stack.erase(stack.begin(), stack.begin() + padding);
        }
    }
}
