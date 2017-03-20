#pragma once

#include <vector>
#include "UtilsDefs.h"
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
	UniformGridsGPU(vector<Point2D> host_points, float radius);

	void gpu_eliminate_conflict_points();
	void gpu_gen_gap_points();
	void gpu_eliminate_gap_points();

	Point2D* dev_points;
	Point2D* dev_gap_points;
	int* dev_idx;

	float rad;
	BBox gridBbox;
	float w_f;
	float h_f;

	int dim;
	int width;
	int height;
};