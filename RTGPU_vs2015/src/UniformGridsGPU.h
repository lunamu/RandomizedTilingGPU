#pragma once

#include <vector>
#include "UtilsDefs.h"
#include <cuda.h>
using namespace std;

struct PointWithHash {
	Point2D p;
	unsigned int hash;
};

struct by_hash {
	bool operator()(PointWithHash const &a, PointWithHash const &b) {
		return a.hash < b.hash;
	}
};


class UniformGridsGPU
{
public:
	UniformGridsGPU() {}
	UniformGridsGPU(vector<Point2D> host_points, int dim_);
	inline unsigned int SpatialToHash(Point2D point) { return (unsigned int)(((int)((point.y - gridBbox.ymin)*height)) * width + (point.x - gridBbox.xmin) * width); }

	void gpu_eliminate_conflict_points();
	void gpu_gen_gap_points();
	void gpu_eliminate_gap_points();

	//Naming: don't name your parameters like functions. Important. Corret later.
	Point2D* dev_points;
	Point2D* dev_gap_points;
	unsigned int* dev_hash;

	int* dev_idx;

	float rad;
	BBox gridBbox;
	float w_f;
	float h_f;

	double itval;
	int dim;
	int width;
	int height;
};