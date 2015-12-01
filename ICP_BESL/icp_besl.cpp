#include "basis.h"
#include "utils.h"
#include "registration.h"

// storage
vector<Vector3f > vertData, colData, normData;
vector<Vector2f > texData;
vector<Vector3f > vertModel, colModel, normModel;
vector<Vector2f > texModel;

int main(int argc, char* argv[])
{
	if (argc != 3)
	{
		cerr << "Invalid arguments" << endl;
		exit(EXIT_FAILURE);
	}

	objLoader(argv[1], vertData, colData, texData, normData);
	objLoader(argv[2], vertModel, colModel, texModel, normModel);

	printObjInfo(vertData, colData, texData, normData);
	printObjInfo(vertModel, colModel, texModel, normModel);

	float thres = 0.1f;
	vector<Vector3f > registredData;
	ICPLoop(vertData, vertModel, registredData, thres);

	objWriter("out_data.obj", registredData);
	
	return 0;
}