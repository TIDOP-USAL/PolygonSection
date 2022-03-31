#pragma once

#include <iostream>
#include <vector>

#include "Vec.h"

class Poly {
private:
	std::vector<Vec3d> vertices;
public:
	Poly(const std::vector<Vec3d>& _vertices) : vertices(_vertices) {}
	Poly() = default;
	~Poly() = default;
private:
	// https://wrf.ecse.rpi.edu/Research/Short_Notes/pnpoly.html
	int pnpoly(int nvert, double* vertx, double* verty, double testx, double testy) {
		int i, j, c = 0;
		for (i = 0, j = nvert - 1; i < nvert; j = i++) {
			if (((verty[i] > testy) != (verty[j] > testy)) &&
				(testx < (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) + vertx[i]))
				c = !c;
		}
		return c;
	}
public:
	// Ignoring z, 2D plane
	bool isPointInside(const Vec3d& point) {
		double* vertx = new double[vertices.size()];
		double* verty = new double[vertices.size()];
		for (int i = 0; i < vertices.size(); i++) {
			vertx[i] = vertices[i].x;
			verty[i] = vertices[i].y;
		}
		int crosses = pnpoly(vertices.size(), vertx, verty, point.x, point.y);
		delete[] vertx;
		delete[] verty;
		return (crosses % 2 == 1) ? true : false;
	}
public:
	inline std::vector<Vec3d>& getVertices() {
		return vertices;
	}
};