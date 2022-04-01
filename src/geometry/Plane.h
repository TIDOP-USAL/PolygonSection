#pragma once

#include <iostream>
#include <sstream>

#include "Vec.h"
#include "Line.h"

//Plane equation: AX + BY + CZ + D = 0
class Plane {
private:
    double A, B, C, D;
    Plane(double _A, double _B, double _C, double _D)
        : A(_A), B(_B), C(_C), D(_D) {
    }
    Plane() = default;
public:
    static Plane plane2vectorsAndPoint(Vec3d& v1, Vec3d& v2, Vec3d& point) {
        Vec3d normal = v1 ^ v2;
        double D = -(point.x * normal.x + point.y * normal.y + point.z * normal.z);
        return Plane(normal.x, normal.y, normal.z, D);
    }

    static Plane plane3points(Vec3d& p1, Vec3d& p2, Vec3d& p3) {
        Vec3d v1 = p2 - p1;
        Vec3d v2 = p3 - p1;
        Vec3d normal = v1 ^ v2;
        double D = -(p1.x * normal.x + p1.y * normal.y + p1.z * normal.z);
        return Plane(normal.x, normal.y, normal.z, D);
    }

    inline bool contains(const Vec3d& point) {
        return ((point.x * A + point.y * B + point.z * C + D) == 0) ? true : false;
    }
public:
    Vec3d lineIntersection(Line& line) {
        double lambda = -( (A * line.getP().x + B * line.getP().y + C * line.getP().z + D) / (A * line.getV().x + B * line.getV().y + C * line.getV().z) );
        return Vec3d(line.getP().x + lambda * line.getV().x, line.getP().y + lambda * line.getV().y, line.getP().z + lambda * line.getV().z);
    }
public:
    inline Vec3d getNormal() {
        return Vec3d(A, B, C);
    }

    inline double getA() const { return A; }
    inline double getB() const { return B; }
    inline double getC() const { return C; }
    inline double getD() const { return D; }

    inline friend std::ostream& operator << (std::ostream& os, const Plane& plane) {
        return os << std::to_string(plane.getA()) + "X + " + std::to_string(plane.getB()) + "Y + " + std::to_string(plane.getC()) + "Z + " +
            std::to_string(plane.getD()) + " = 0";
    }
};