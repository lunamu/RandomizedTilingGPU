#pragma once

#include "UtilsDefs.h"

template <typename T>
void viewGPUArray(T* array, int num, string filename)
{
	const string dir = "C:/Users/lunam/Dropbox/MATLAB/";
	T* host_array = new T[num];
	GPUCHECK(cudaMemcpy(host_array, array, sizeof(T) * num, cudaMemcpyDeviceToHost));
	ofstream file(dir + filename);
	for (int i = 0; i < num; i++)
	{
		file << host_array[i] << endl;
	}
	file.close();
	delete[] host_array;
}