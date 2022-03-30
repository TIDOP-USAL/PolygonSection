#pragma once

#include "Vec.h"

/*

    Line equation:

        r = (x0, y0, z0) + t(a, b, c)

    Parametric equations:
            _
            |   x = x0 + at
        r   |   y = y0 + bt
            |   z = z0 + bt
            -

    Where p = (x0, y0, z0) is a point and v = (a, b, c) is a vector
    we can obtain (a, b, c) from 2 given points, and consider (x0, y0, z0)
    the coordinates of one of those 2 points
*/
class Line {
private:
    Vec3d p, v;
    Line(const Vec3d& _p, const Vec3d& _v)
        : p(_p), v(_v) {
    }
public:

    Line() = default;

    static Line from2points(const Vec3d& p1, const Vec3d& p2) {
        Vec3d v(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
        return Line(p1, v);
    }

    static Line fromPointVector(const Vec3d& p, const Vec3d& v) {
        return Line(p, v);
    }
public:
    inline Vec3d& getP() {
        return p;
    }

    inline Vec3d& getV() {
        return v;
    }
};