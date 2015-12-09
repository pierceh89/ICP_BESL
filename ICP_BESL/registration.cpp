#include "basis.h"
#include "registration.h"
#include "timeLapse.h"

void centroid(const vector<Vector3f >& vertices, MatrixXf& ret, vector<size_t>* idx)
{
	float x(0.0), y(0.0), z(0.0);
	float invSize;
	if (idx == nullptr)
	{
		for (size_t i = 0; i < vertices.size(); i++)
		{
			x += vertices[i](0, 0);
			y += vertices[i](1, 0);
			z += vertices[i](2, 0);
		}
		invSize = 1.f / static_cast<float>(vertices.size());
	}
	else
	{
		for (size_t i = 0; i < idx->size(); i++)
		{
			x += vertices[idx->at(i)](0,0);
			y += vertices[idx->at(i)](1,0);
			z += vertices[idx->at(i)](2,0);
		}
		invSize = 1.f / static_cast<float>(idx->size());
	}
	ret(0, 0) = x*invSize;
	ret(1, 0) = y*invSize;
	ret(2, 0) = z*invSize;
}

void xcov(const vector<Vector3f >& vd, const MatrixXf& ctd, const vector<Vector3f >& vm, const MatrixXf& ctm, const vector<size_t>& idx, Matrix3f& ret)
{
	size_t szData = vd.size();
	Matrix3f xcovret;
	xcovret.setZero();
	Matrix3f covct = (ctd*ctm.transpose()).topLeftCorner<3,3>();

	for (size_t i = 0; i < szData; i++)
	{
		xcovret(0, 0) += vd[i](0) * vm[idx[i]](0) - covct(0,0);
		xcovret(0, 1) += vd[i](0) * vm[idx[i]](1) - covct(0,1);
		xcovret(0, 2) += vd[i](0) * vm[idx[i]](2) - covct(0,2);
		xcovret(1, 0) += vd[i](1) * vm[idx[i]](0) - covct(1,0);
		xcovret(1, 1) += vd[i](1) * vm[idx[i]](1) - covct(1,1);
		xcovret(1, 2) += vd[i](1) * vm[idx[i]](2) - covct(1,2);
		xcovret(2, 0) += vd[i](2) * vm[idx[i]](0) - covct(2,0);
		xcovret(2, 1) += vd[i](2) * vm[idx[i]](1) - covct(2,1);
		xcovret(2, 2) += vd[i](2) * vm[idx[i]](2) - covct(2,2);
	}

	xcovret = xcovret / static_cast<float>(szData);
	ret << xcovret;
}

void computeQ(const Matrix3f& xcovMat, Matrix4f& ret)
{
	float trCov = xcovMat.trace();
	Matrix3f antiSym = xcovMat - xcovMat.transpose();
	MatrixXf delta(3, 1);
	delta(0, 0) = antiSym(1, 2);
	delta(1, 0) = antiSym(2, 0);
	delta(2, 0) = antiSym(0, 1);
	Matrix3f tmpMat = xcovMat + xcovMat.transpose() - trCov*Matrix3f::Identity();
	ret << trCov, delta.transpose(),
		delta, tmpMat;
}

void computeRT(const Matrix4f& Q, const MatrixXf& ctData, const MatrixXf& ctModel, Matrix3f& retR, MatrixXf& retT)
{
	EigenSolver<Matrix4f> es(Q);
	//cout << es.eigenvalues() << endl;
	size_t maxI = 0;
	float maxEigen = es.eigenvalues()[0].real();
	for (size_t i = 1; i < 4; i++)
	{
		if (maxEigen < es.eigenvalues()[i].real() )
		{
			maxEigen = es.eigenvalues()[i].real();
			maxI = i;
		}
	}
	//cout << "maxI: " << maxI << endl;
	float q[4];
	q[0] = es.eigenvectors().col(maxI)[0].real();
	q[1] = es.eigenvectors().col(maxI)[1].real();
	q[2] = es.eigenvectors().col(maxI)[2].real();
	q[3] = es.eigenvectors().col(maxI)[3].real();

	float sqrQ0 = q[0] * q[0], sqrQ1 = q[1] * q[1], sqrQ2 = q[2] * q[2], sqrQ3 = q[3] * q[3],
		q0q1 = q[0] * q[1], q0q2 = q[0] * q[2], q0q3 = q[0] * q[3],
		q1q2 = q[1] * q[2], q1q3 = q[1] * q[3],
		q2q3 = q[2] * q[3];

	retR(0, 0) = sqrQ0 + sqrQ1 - sqrQ2 - sqrQ3;
	retR(0, 1) = 2 * (q1q2 - q0q3);
	retR(0, 2) = 2 * (q1q3 + q0q2);
	retR(1, 0) = 2 * (q1q2 + q0q3);
	retR(1, 1) = sqrQ0 + sqrQ2 - sqrQ1 - sqrQ3;
	retR(1, 2) = 2 * (q2q3 - q0q1);
	retR(2, 0) = 2 * (q1q3 - q0q2);
	retR(2, 1) = 2 * (q2q3 + q0q1);
	retR(2, 2) = sqrQ0 + sqrQ3 - sqrQ1 - sqrQ2;

	retT = ctModel - retR*ctData;
}

void naiveCorrespondence(const vector<Vector3f >& data, const vector<Vector3f >& model, vector<size_t>& idx)
{
	size_t szData = data.size(), szModel = model.size();
	size_t minI;
	float minDistance;
	for (size_t i = 0; i < szData; i++)
	{
		minI = 0;
		minDistance = (data[i] - model[0]).squaredNorm();
		for (size_t j = 1; j < szModel; j++)
		{
			float sqrNorm = (data[i] - model[j]).squaredNorm();
			if (minDistance > sqrNorm)
			{
				minDistance = sqrNorm;
				minI = j;
			}
		}
		idx.push_back(minI);
	}
}

void ICPLoop(const vector<Vector3f >& data, const vector<Vector3f >& model, vector<Vector3f >& registratedData, float threshold)
{
	vector<size_t> idx;
	vector<Vector3f > curdata(data);
	float tolerance = 0.01;
	float prevRMSE = 10e5, RMSE;
	do{
		// compute closest point
		idx.clear();
		//naiveCorrespondence(curdata, model, idx);
		cout << "Elapsed Time finding correspondences: "
			<< tim::measure<>::execution(naiveCorrespondence, curdata, model, idx) << " miliseconds" << endl;

		// compute centroid
		MatrixXf ctData(3, 1), ctModel(3, 1);
		centroid(curdata, ctData);
		centroid(model, ctModel, &idx);

		cout << "Centroid of Data: " << endl << ctData << endl;
		cout << "Centroid of Model: " << endl << ctModel << endl;

		// compute cross covariance matrix
		Matrix3f xcovmat;
		xcov(curdata, ctData, model, ctModel, idx, xcovmat);
		cout << "Cross covariance: " << endl << xcovmat << endl;

		Matrix4f Qmat;
		computeQ(xcovmat, Qmat);
		cout << "Qmatrix: " << endl << Qmat << endl;

		// compute Rotation and Translation
		Matrix3f R;
		MatrixXf T(3, 1);
		computeRT(Qmat, ctData, ctModel, R, T);
		cout << "Rotation: " << endl << R << endl;
		cout << "Translation: " << endl << T << endl;

		// transform the data
		for (size_t i = 0; i < curdata.size(); i++)
		{
			curdata[i] = R * curdata[i] + T;
		}

		// compute RMSE
		RMSE = computeRMSE(curdata, model, idx);
		cout << "RMSE: " << RMSE << endl;

		float diff = prevRMSE - RMSE;
		if (diff < tolerance || RMSE < threshold)
			break;

		prevRMSE = RMSE;
	} while (true);
	registratedData = curdata;
}