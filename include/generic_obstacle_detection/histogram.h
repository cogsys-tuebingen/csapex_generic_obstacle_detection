#ifndef HISTOGRAM_H
#define HISTOGRAM_H

namespace csapex
{

class Histogram
{
public:
    struct Params {
        Params(unsigned w, unsigned h, double ox, double oy, double resolution)
            : w(w), h(h),ox(ox),oy(oy),resolution(resolution)
        {}
        Params() {}

        unsigned w;
        unsigned h;
        double ox;
        double oy;
        double resolution;
    };

public:
    Histogram(const Params& params);

    int mapX(double x) const;
    int mapY(double y) const;
    double unmapX(int x) const;
    double unmapY(int y) const;

    bool inside(int x, int y) const;

    unsigned index(unsigned x, unsigned y) const;

    unsigned width() const;
    unsigned height() const;

    double oX() const;
    double oY() const;
    double resolution() const;

protected:
    unsigned w_;
    unsigned h_;

    double ox_;
    double oy_;
    double resolution_;
};

}

#endif // HISTOGRAM_H
