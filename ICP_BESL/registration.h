#ifndef H_REGISTRATION
#define H_REGISTRATION
#include "basis.h"

void centroid(const vector<Vector3f >& vertices, MatrixXf& ret, vector<size_t>* idx=nullptr);
void xcov(const vector<Vector3f >& vd, const MatrixXf& ctd, const vector<Vector3f >& vm, const MatrixXf& ctm, const vector<size_t>& idx, Matrix3f& ret);
void computeQ(const Matrix3f& xcovMat, Matrix4f& ret);
void computeRT(const Matrix4f& Q, const MatrixXf& ctData, const MatrixXf& ctModel, Matrix3f& retR, MatrixXf& retT);
void naiveCorrespondence(const vector<Vector3f >& data, const vector<Vector3f >& model, vector<size_t>& idx);
void ICPLoop(const vector<Vector3f >& data, const vector<Vector3f >& model, vector<Vector3f >& registratedData, float threshold);
inline float computeRMSE(const vector<Vector3f>& data, const vector<Vector3f >& model, const vector<size_t>& idx)
{
	float v = 0.0;
	for (size_t i = 0; i < idx.size(); i++)
		v = v + (data[i] - model[i]).norm();
	v = v / static_cast<float>(idx.size());
	return v;
}

#endif