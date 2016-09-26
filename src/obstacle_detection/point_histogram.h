#ifndef POINT_HISTOGRAM_H
#define POINT_HISTOGRAM_H

/// COMPONENT
#include <patsy/histogram.h>

/// SYSTEM
#include <vector>

namespace csapex {

template <class PointT>
class PointHistogram : public Histogram
{
public:
    PointHistogram(const Histogram::Params& params);
    void reset();

    void insert(const PointT& pt, unsigned int idx);

//    inline PointT& operator () (unsigned x, unsigned y, unsigned stack) {
//        return points_[y * w_ + h_][stack];
//    }
    inline std::vector<std::pair<PointT,unsigned> >& operator [] (unsigned idx) {
        return points_[idx];
    }

    void sort();



    int size() {
        return points_.size();
    }

    unsigned getMinX() {
        return minx;
    }

    unsigned getMinY() {
        return miny;
    }

    unsigned getMaxX() {
        return maxx;
    }

    unsigned getMaxY() {
        return maxy;
    }

private:
    std::vector<std::vector<std::pair<PointT,unsigned int> > > points_;

    unsigned minx;
    unsigned maxx;
    unsigned miny;
    unsigned maxy;
};

}
#endif // POINT_HISTOGRAM_H
