#pragma once

#include <iostream>
#include <cmath>
#include <sstream>

#define M_E        2.71828182845904523536   // e
#define M_LOG2E    1.44269504088896340736   // log2(e)
#define M_LOG10E   0.434294481903251827651  // log10(e)
#define M_LN2      0.693147180559945309417  // ln(2)
#define M_LN10     2.30258509299404568402   // ln(10)
#define M_PI       3.14159265358979323846   // pi
#define M_PI_2     1.57079632679489661923   // pi/2
#define M_PI_4     0.785398163397448309616  // pi/4
#define M_1_PI     0.318309886183790671538  // 1/pi
#define M_2_PI     0.636619772367581343076  // 2/pi
#define M_2_SQRTPI 1.12837916709551257390   // 2/sqrt(pi)
#define M_SQRT2    1.41421356237309504880   // sqrt(2)
#define M_SQRT1_2  0.707106781186547524401  // 1/sqrt(2)

template <typename T>
struct Vec3 {

    union { T x; T r; T u; };
    union { T y; T g; T v; };
    union { T z; T b; T w; };

    Vec3(T _x, T _y, T _z)
        : x(_x), y(_y), z(_z) {
    }

    Vec3() = default;
    ~Vec3() = default;

    inline double getModule() {
        return sqrt(x * x + y * y + z * z);
    }

    Vec3 getUnit() {
        double norm = getModule();
        return Vec3(x / norm, y / norm, z / norm);
    }

    // Dot product
    inline double operator * (const Vec3& vec) {
        return vec.x * x + vec.y * y + vec.z * z;
    }

    Vec3& operator * (T value) {
        x *= value;
        y *= value;
        z *= value;
        return *this;
    }

    // Cross product
    inline Vec3 operator ^ (const Vec3& vec) {
        return Vec3(y * vec.z - z * vec.y, x * vec.z - z * vec.x, x * vec.y - y * vec.x);
    }

    inline Vec3 operator + (const Vec3& vec) {
        return Vec3(vec.x + x, vec.y + y, vec.z + z);
    }

    inline Vec3 operator - (const Vec3& vec) {
        return Vec3(vec.x - x, vec.y - y, vec.z - z);
    }
};

typedef Vec3<float> Vec3f;
typedef Vec3<double> Vec3d;
typedef Vec3<int> Vec3i;

template <typename T>
struct Vec4 {

    Vec3<T> vec;
    union { T w; T a; };

    Vec4(const Vec3<T>& _vec, T _w)
        : vec(_vec), w(_w) {
    }

    Vec4 operator * (const std::vector<std::vector<double>>& matrix) {
        T x = matrix[0][0] * vec.x + matrix[0][1] * vec.y + matrix[0][2] * vec.z + matrix[0][3] * vec.w;
        T y = matrix[1][0] * vec.x + matrix[1][1] * vec.y + matrix[1][2] * vec.z + matrix[1][3] * vec.w;
        T z = matrix[2][0] * vec.x + matrix[2][1] * vec.y + matrix[2][2] * vec.z + matrix[2][3] * vec.w;
        T w = matrix[3][0] * vec.x + matrix[3][1] * vec.y + matrix[3][2] * vec.z + matrix[3][3] * vec.w;
        return Vec4(Vec3<T>(x, y, z), w);
    };
};

typedef Vec4<float> Vec4f;
typedef Vec4<double> Vec4d;
typedef Vec4<int> Vec4i;