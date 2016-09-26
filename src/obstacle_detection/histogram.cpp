/// HEADER
#include <generic_obstacle_detection/histogram.h>

using namespace csapex;

Histogram::Histogram(const Params &params)
    : w_(params.w), h_(params.h), ox_(params.ox), oy_(params.oy), resolution_(params.resolution)
{
}

int Histogram::mapX(double x) const
{
    return (x + ox_) / resolution_;
}

int Histogram::mapY(double y) const
{
    return (y + oy_) / resolution_;
}

double Histogram::unmapX(int x) const
{
    return (x * resolution_) - ox_;
}

double Histogram::unmapY(int y) const
{
    return (y * resolution_) - oy_;
}

bool Histogram::inside(int x, int y) const
{
    unsigned ux = x;
    unsigned uy = y;

    if(x < 0 || ux >= w_ || y < 0 || uy >= h_) {
        return false;
    }

    return true;
}

unsigned Histogram::index(unsigned x, unsigned y) const
{
    return y * w_ + x;
}

unsigned Histogram::width() const
{
    return w_;
}

unsigned Histogram::height() const
{
    return h_;
}

double Histogram::oX() const
{
    return ox_;
}

double Histogram::oY() const
{
    return oy_;
}

double Histogram::resolution() const
{
    return resolution_;
}
