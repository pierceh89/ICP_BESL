#include "basis.h"
#include "utils.h"

void objLoader(const string& fname, vector<Vector3f >& vertices, vector<Vector3f >& color,
	vector<Vector2f >& tex, vector<Vector3f >& normals)
{
	fstream file;
	const unsigned int len = 256;
	char line[len];
	file.open(fname.c_str());
	if (!file.is_open())
	{
		cerr << "Cannot open File" << endl;
		exit(0);
	}
	while (file.getline(line, len))
	{
		if (line[0] == 'v')
		{
			stringstream ss(line + 2);
			switch (line[1])
			{
			case 't':
			{
				Vector2f t;
				ss >> t[0] >> t[1];
				tex.push_back(t);
			}
			break;

			case 'n':
			{
				Vector3f n;
				ss >> n[0] >> n[1] >> n[2];
				normals.push_back(n);
			}
			break;

			case ' ':
			{
				Vector3f v;
				ss >> v[0] >> v[1] >> v[2];
				vertices.push_back(v);
				Vector3f c;
				ss >> c[0] >> c[1] >> c[2];
				color.push_back(c);
			}

			break;
			}
		}
	}
}

void objWriter(const string& fname, const vector<Vector3f >& points)
{
	std::ofstream file;
	file.open(fname.c_str());
	if (!file.is_open())
	{
		cerr << "Writing obj: file open error" << endl;
		return;
	}
	for (size_t i = 0; i < points.size(); i++)
		file << "v " << points[i](0) << " " << points[i](1) << " " << points[i](2) << " " << 1 << " " << 1 << " " << 0 << endl;

	file.close();
}