#ifndef H_UTILS
#define H_UTILS
#include "basis.h"

void objLoader(const string& fname, vector<Vector3f >& vertices, vector<Vector3f >& color,
	vector<Vector2f >& tex, vector<Vector3f >& normals);

void objWriter(const string& fname, const vector<Vector3f >& points);

inline void printObjInfo(const vector<Vector3f >& vert, const vector<Vector3f >& col, const vector<Vector2f >& tex, const vector<Vector3f >& norm)
{
	cout << "# of vertex | # of color | # of texture | # of normal  " << endl;
	cout << "    " << vert.size() << "    " << col.size() << "    " << tex.size() << "    " << norm.size() << endl;
}

#endif