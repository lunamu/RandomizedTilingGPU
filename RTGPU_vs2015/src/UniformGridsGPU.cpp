#include "UniformGridsGPU.h"

UniformGridsGPU::UniformGridsGPU(vector<Point2D> host_points, int dim_)
{
	//Set some class parameters
	dim = dim_;
	gridBbox = gUnitGrid;
	w_f = gridBbox.xmax - gridBbox.xmin;
	h_f = gridBbox.ymax - gridBbox.ymin;
	itval = 1.0 / dim;
	width = dim_;
	height = dim_;

	//allocate GPU memory;
	gpuErrchk(cudaMalloc((void**)&dev_points, sizeof(PointWithHash) * host_points.size()));
	gpuErrchk(cudaMalloc((void**)&dev_idx, sizeof(int) * host_points.size()));

	//Modify: This might not be correct
	gpuErrchk(cudaMalloc((void**)&dev_points, sizeof(int) * host_points.size()));

	//Generate hash for all points
	

}
