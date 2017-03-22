#include "UniformGridsGPU.h"



__global__ void UniformGridsGPU::cuda_generate_hash_value(int size)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx < size)
	{
		dev_php[idx].p = dev_points[idx];
		dev_php[idx].priority = dev_priority[idx];
		dev_php[idx].valid = false;
		int hash = SpatialToHash(dev_points[idx]);
		dev_php[idx].hash = hash;
		dev_hash[idx] = hash;
		dev_valid[idx] = true;
		if (idx == 0)dev_mark[idx] = dev_hash[idx];
		else
		{
			if (dev_hash[idx] == dev_hash[idx - 1])dev_mark[idx] = -1;
			else dev_mark[idx] = dev_hash[idx];
		}
	}
}

__global__ void UniformGridsGPU::cuda_init_index(int size)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx < size)
	{
		dev_idx[idx] = -1;
	}
}

__global__ void UniformGridsGPU::cuda_generate_index(int size)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx < size)
	{
		if (dev_mark[idx] != -1)dev_idx[dev_mark[idx]] = idx;
	}
}

__global__ void UniformGridsGPU::cuda_eliminate_conflict(float range, int size)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx < size)
	{
		int cp_index_buffer[MAX_LOCAL_NUM];
		eliminate_conflict(idx, range, cp_index_buffer);
	}
}

const double EPS = 0.000001;
UniformGridsGPU::UniformGridsGPU(vector<Point2D>& host_points, vector<int>& host_priority, int dim_)
{
	//Set some class parameters
	dim = dim_;
	gridBbox = gUnitGrid;
	w_f = gridBbox.xmax - gridBbox.xmin;
	h_f = gridBbox.ymax - gridBbox.ymin;
	itval = 1.0 / dim;
	width = dim_;
	height = dim_;

	point_num = host_points.size();
	//allocate GPU memory;
	gpuErrchk(cudaMalloc((void**)&dev_points, sizeof(PointWithHash) * host_points.size()));
	gpuErrchk(cudaMalloc((void**)&dev_idx, sizeof(int) * width * height));
	gpuErrchk(cudaMalloc((void**)&dev_hash, sizeof(int) * host_points.size()));
	gpuErrchk(cudaMalloc((void**)&dev_priority, sizeof(int) * host_priority.size()));
	gpuErrchk(cudaMalloc((void**)&dev_php, sizeof(PointStruct) * host_points.size()));
	gpuErrchk(cudaMalloc((void**)&dev_valid, sizeof(int) * point_num));
	gpuErrchk(cudaMalloc((void**)&dev_mark, sizeof(int) * point_num));


	//copy points and priority from host
	gpuErrchk(cudaMemcpy(dev_points, &host_points[0], sizeof(Point2D) * point_num, cudaMemcpyHostToDevice));
	gpuErrchk(cudaMemcpy(dev_priority, &dev_priority[0], sizeof(int) * point_num, cudaMemcpyHostToDevice))


	//Modify: This might not be correct

	//Generate hash for all points
	int threadsPerBlock = 256;
	int numBlocks = (point_num + threadsPerBlock - 1) / threadsPerBlock;
	cuda_generate_hash_value << <numBlocks, threadsPerBlock >> >(host_points.size());

	//Sort
	
	
	//backup hashvalue array
	int* dev_hash_backup;
	gpuErrchk(cudaMalloc((void**)&dev_hash_backup, sizeof(int) * host_points.size()));
	gpuErrchk(cudaMemcpy(dev_hash_backup, dev_hash, sizeof(int) * point_num, cudaMemcpyDeviceToDevice));

	thrust::device_ptr<int> dev_hash_pointer(dev_hash);
	thrust::device_ptr<int> ddev_hash_backup_pointer(dev_hash_backup);
	thrust::device_ptr<PointStruct> dev_php_pointer(dev_php);
	thrust::sort_by_key(thrust::device, dev_hash_pointer, dev_hash_pointer + point_num, dev_points);	
	thrust::sort_by_key(thrust::device, ddev_hash_backup_pointer, ddev_hash_backup_pointer + point_num, dev_php);


	//Generate idx
	cuda_init_index << <numBlocks, threadsPerBlock >> >(point_num);
	cuda_generate_index << <numBlocks, threadsPerBlock >> >(point_num);

}

__device__ bool UniformGridsGPU::dart_search(Point2D query, float range)
{
	unsigned int x_start = max(0, (query.x - range - gridBbox.xmin)) * width;
	unsigned int x_end = min(gridBbox.xmax - EPS - gridBbox.xmin, (query.x + range - gridBbox.xmin)) * width;
	unsigned int y_start = max(0, (query.y - range - gridBbox.ymin)) * height;
	unsigned int y_end = min(gridBbox.ymax - EPS - gridBbox.ymin, (query.y + range - gridBbox.ymin)) * height;
	for (int i = x_start; i <= x_end; i++)
	{
		for (int j = y_start; j <= y_end; j++)
		{
			unsigned int idx = j * width + i;
			if (dev_idx[idx] == -1)
			{
				continue;
			}
			else
			{
				int grid_index = dev_idx[idx];
				int grid_hash = dev_hash[idx];
				int in_grid_idx = 0;
				while ((grid_index + in_grid_idx < point_num) && (dev_hash[grid_index + in_grid_idx] == grid_hash)) {
					
					double distSquare = DistanceSquared(query, dev_points[grid_index + in_grid_idx]);
					if (distSquare == 0.0)continue;
					//if (distSquare < 1e-20)continue;
					else if (distSquare < range * range)
					{
						return false;
					}
					in_grid_idx++;
				}
			
			}
		}
	}
	return true;

}

__device__ bool UniformGridsGPU::eliminate_conflict(int query_idx, float range, int* cp_index_buffer)//conflict points index buffer.
{
	
	int p_num = 0;
	Point2D query = dev_points[query_idx];
	unsigned int x_start = max(0, (query.x - range - gridBbox.xmin)) * width;
	unsigned int x_end = min(gridBbox.xmax - EPS - gridBbox.xmin, (query.x + range - gridBbox.xmin)) * width;
	unsigned int y_start = max(0, (query.y - range - gridBbox.ymin)) * height;
	unsigned int y_end = min(gridBbox.ymax - EPS - gridBbox.ymin, (query.y + range - gridBbox.ymin)) * height;
	for (int i = x_start; i <= x_end; i++)
	{
		for (int j = y_start; j <= y_end; j++)
		{
			unsigned int idx = j * width + i;
			if (dev_idx[idx] == -1)
			{
				continue;
			}
			else
			{
				int current_priority = dev_priority[query_idx];
				int grid_index = dev_idx[idx];
				int grid_hash = dev_hash[idx];
				int in_grid_idx = 0;
				while ((grid_index + in_grid_idx < point_num) && (dev_hash[grid_index + in_grid_idx] == grid_hash)) {
					if (dev_priority[grid_index + in_grid_idx] > current_priority)
					{
						eliminate_conflict(grid_index + in_grid_idx, range, cp_index_buffer);
						return;
					}
					else
					{
						cp_index_buffer[p_num++] = grid_index + in_grid_idx;
					}
					in_grid_idx++;
				}
			}
		}
	}

	if (p_num > 0)
	{
		for (int i = 0; i < p_num; i++)
		{
			//eliminate the point;
			if (cp_index_buffer[i] != query_idx) { dev_valid[cp_index_buffer[i]] = false; }
		}
	}
	return true;
}

void UniformGridsGPU::gpu_eliminate_conflict_points(float range)
{
	int threadsPerBlock = 256;
	int numBlocks = (point_num + threadsPerBlock - 1) / threadsPerBlock;
	cuda_eliminate_conflict << <numBlocks, threadsPerBlock >> > (range, point_num);
}
