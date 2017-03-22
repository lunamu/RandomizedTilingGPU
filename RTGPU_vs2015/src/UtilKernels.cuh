#pragma once

#include "UtilsDefs.h"
//
//template <typename T>
//void viewGPUArray(T* array, int num, string filename)
//{
//	const string dir = "C:/Users/lunam/Dropbox/MATLAB/";
//	T* host_array = new T[num];
//	gpuErrchk( cudaMemcpy(host_array, array, sizeof(T) * num, cudaMemcpyDeviceToHost)));
//	ofstream file(dir + filename);
//	for (int i = 0; i < num; i++)
//	{
//		file << host_array[i] << endl;
//	}
//	file.close();
//	delete[] host_array;
//}

void viewGPUPoint2D(Point2D* points, int num, string filename)
{
	const string dir = "C:/Users/lunam/Dropbox/MATLAB/";
	Point2D* host_array = new Point2D[num];
	gpuErrchk(cudaMemcpy(host_array, points, sizeof(Point2D) * num, cudaMemcpyDeviceToHost));

	ofstream file(dir + filename);
	for (int i = 0; i < num; i++)
	{
		file << host_array[i].x <<" "<<host_array[i].y << endl;
	}
	file.close();
	delete[] host_array;
}