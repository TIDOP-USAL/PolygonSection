#pragma once
#include <iostream>
#include <vector>
#include <cmath>

#include "Vec.h"

enum class Axis {
    X, Y, Z
};

class Transform4x4 {
public:
    static std::vector<std::vector<double>> rotationMatrix(const Axis& axis, double angle) {
        std::vector<std::vector<double>> matrix;
        switch (axis) {
        case Axis::X:
            matrix = {
                { 1,     0,           0,      0 },
                { 0, cos(angle), -sin(angle), 0 },
                { 0, sin(angle),  cos(angle), 0 },
                { 0,     0,           0,      1 }
            };
            break;
        case Axis::Y:
            matrix = {
                { cos(angle), 0, -sin(angle), 0 },
                {     0,      1,       0,     0 },
                { sin(angle), 0, cos(angle),  0 },
                {     0,      0,       0,     1 }
            };
            break;
        case Axis::Z:
            matrix = {
                { cos(angle), -sin(angle), 0, 0 },
                { sin(angle), cos(angle),  0, 0 },
                {     0,          0,       1, 0 },
                {     0,          0,       0, 1 }
            };
            break;
        }
        return matrix;
    }

    static std::vector<std::vector<double>> scaleMatrix(double scale) {
        return {
            { scale,  0,    0,    0 },
            {   0,  scale,  0,    0 },
            {   0,    0,  scale,  0 },
            {   0,    0,    0,    1 }
        };
    }

    static std::vector<std::vector<double>> translationMatrix(double tx, double ty, double tz) {
        return {
            { 1, 0, 0, tx },
            { 0, 1, 0, ty },
            { 0, 0, 1, tz },
            { 0, 0, 0,  1 }
        };
    }
};