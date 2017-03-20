//A 2D DNC Poisson Disk Sampling Method on GPU.
//Make it fast


#include "UtilsDefs.h"
#include "KD_tree.h"
#include "UniformGrids.h"
#include "RandomizedTiling.h"
#include "Tiller.h"

using namespace std;


vector<Point2D> primaryPoints;
vector<Point2D> secondaryPoints;
BBox primary_right;
BBox primary_down;
BBox secondary_left;
BBox secondary_up;
BBox mid_NE;
BBox mid_SW;

vector<Point2D> centerPoints;
vector<Point2D> triPoints;

//This is storing blockingBox for visulization.
vector<BBox> blockingBox;
KDnode* kd_tree;
//argv explained:
//argv[0]: excutable
//argv[1]: filename of the pattern input
//argv[2]: filename of the maximal poisson disk sampling result




int main(int argc, char* argv[])
{
	float ratio = atof(argv[3]);
	double radius = 0.005 / ratio;
	//srand((unsigned)time(NULL));
	string pattern_filename = argv[1];
	string result_filename = argv[2];
	ifstream fin_pattern(pattern_filename);
	

	//build kd-tree for pattern
	vector<Point2D> pattern;
	Point2D tmp; while ((fin_pattern >> tmp.x, fin_pattern >> tmp.y))pattern.push_back(tmp);
	kd_tree = build(&pattern[0], pattern.size(), 0);

	
	//First, traverse and record all priority.
	BBox original_bbox;
	original_bbox.xmin = 0.0; original_bbox.xmax = 1.0; original_bbox.ymin = 0.0; original_bbox.ymax = 1.0;
	
	

	//Tile them all in one UniformGrid



	clock_t start = clock(), diff;
	//Timing
	Tiller tiller(original_bbox, kd_tree, radius);
	tiller.DivideConquerTiling(original_bbox, radius, XAXIS, ratio);
	//DNCPoisson(original_bbox, points, priority, radius, XAXIS,ratio);
	
	//TODO:GPU Uniform Grid
	//UniformGridsGPU

	diff = clock() - start;
	int msec = diff * 1000 / CLOCKS_PER_SEC;
	printf("Ratio: %f\nDNC taken %d seconds %d milliseconds\n", ratio, msec / 1000, msec % 1000);
	//printf("%f %d ", ratio, msec);

	
	for (int i = 0; i < tiller.result.size(); i++)
	{
		tiller.points_in_grid.insert(tiller.result[i], tiller.priority[i]);
	}


	start = clock(), diff;
	//Timing
	tiller.eliminate_for_maximal(radius);
	//points_in_grid.eliminate_for_maximal_batch(radius);


	int count = tiller.getGridCount();
	diff = clock() - start;
	msec = diff * 1000 / CLOCKS_PER_SEC;
	//printf("%d %d\n", count, msec);
	printf("Points:%d\nEliminate for Maximal take %d seconds %d milliseconds\n",count, msec / 1000, msec % 1000);

	tiller.printToFile(result_filename);


	//points_in_grid.test_maximal(radius);

	UniformGrids ugrids(1.0 / (2 * 0.005));
	for (int i = 0; i < pattern.size(); i++)
	{
		ugrids.insert(pattern[i]);
	}
	ugrids.test_maximal(0.005);


	cin >> diff;

}