/*
Position module
This module localizes the boat by outputting boat's position
*/
#ifndef __POSITION_H
#define __POSITION_H

// Pre-defined variable for the number of total clusters in Kmeans
#define k 1

struct Tuple {
	float attr1;
	float attr2;
};
vector<Tuple> tuples;
vector<Tuple> tuplesY;
vector<Tuple> tuplesG;
vector<Point> tuplesPoints;
vector<Point> tuplesPointsY;
vector<Point> tuplesPointsG;
Tuple resultMeans[k];
Tuple resultMeansY[k];
Tuple resultMeansG[k];
// These are variables for kalman filtering
vector<Point> resultMeansPoints;
vector<Point> resultMeansPointsY;
vector<Point> resultMeansPointsG;
vector<Point> median;
vector<Point> medianY;
vector<Point> medianG;
vector<Point> medianKal;
vector<Point> boatPointR[k];
vector<Point> boatPointK[k];
//HSV
CvScalar s1;
CvScalar s2;
CvScalar s3;

Tuple tupleY;

//Kalman filtering
double kg = 0;
double Xk_kp1 = 0;
double pkp1_kp1 = 0;		// p(k-1|k-1)
double pk_k = 0;		    // p(k|k)
double pk_kp1 = 0;		    // p(k|k-1)
double kg_y = 0;
double Xk_kp1_y = 0;
double pkp1_kp1_y = 0;		// p(k-1|k-1)
double pk_k_y = 0;		    // p(k|k)
double pk_kp1_y = 0;		// p(k|k-1)

int flag_num = 0;
int flag_num_boatPointR = 0;

/* Function prototypes */
vector<Tuple> yellowColor(vector<Tuple> tupleVector, IplImage *pBlack, int i, int j, CvScalar s3, uchar B, uchar G, uchar R);
Tuple redColor(IplImage *pBlack, int i, int j, CvScalar s3, uchar B, uchar G, uchar R);
float getDistXY(Tuple t1, Tuple t2);
int getPointXY(Point, Point);
int clusterOfTuple(Tuple means[], Tuple tuple);
float getVar(vector<Tuple> clusters[], Tuple means[]);
Tuple getMeans(vector<Tuple> cluster);
Tuple KMeans(vector<Tuple> tuples);
vector<Point> kalmanFilter(vector<Point> mea, vector<Point> meaRes,  double cova, double Q, double R);


//------------------------for color-------------------------
//Yellow
vector<Tuple> yellowColor(vector<Tuple> tupleVector, IplImage *pBlack, int i, int j, CvScalar s3, uchar B, uchar G, uchar R)
{
	Tuple tuple;
	if ((abs(s3.val[0] - 65) < 10
		&& s3.val[1] > 0.8)
		&&
		!((abs(double(R) - 0) < 10)
			&& abs(double(G) - 0) < 10
			&& abs(double(B) - 0) < 10))
	{
		tuple.attr1 = i;
		tuple.attr2 = j;
		tupleVector.push_back(tuple);

		Point points;
		points.x = i;
		points.y = j;
		//tuplesPointsY.push_back(points);
		CV_IMAGE_ELEM(pBlack, uchar, j, 3 * i) = 0;
		CV_IMAGE_ELEM(pBlack, uchar, j, 3 * i + 1) = 255;
		CV_IMAGE_ELEM(pBlack, uchar, j, 3 * i + 2) = 255;
	}
	return tupleVector;
}
//Red
Tuple redColor(IplImage *pBlack, int i, int j, CvScalar s3, uchar B, uchar G, uchar R)
{
	Tuple tuple;
	if ((20 - s3.val[0] > 0
		|| 360 - s3.val[0]  < 20
		&& s3.val[1] > 0.25)
		&&
		!((abs(double(R) - 0) < 10)
			&& abs(double(G) - 0) < 10
			&& abs(double(B) - 0) < 10))
	{
		tuple.attr1 = i;
		tuple.attr2 = j;
		//tupleVector.push_back(tuple);

		Point points;
		points.x = i;
		points.y = j;
		//tuplesPointsG.push_back(points);
		CV_IMAGE_ELEM(pBlack, uchar, j, 3 * i) = 0;
		CV_IMAGE_ELEM(pBlack, uchar, j, 3 * i + 1) = 0;
		CV_IMAGE_ELEM(pBlack, uchar, j, 3 * i + 2) = 255;

		return tuple;
	}
}
//------------------------Kmeans----------------------------
// Calculate the Eucledian distance between tuples  
float getDistXY(Tuple t1, Tuple t2)
{
	return sqrt((t1.attr1 - t2.attr1) * (t1.attr1 - t2.attr1) + (t1.attr2 - t2.attr2) * (t1.attr2 - t2.attr2));
}
//Calculate the pixel between two point
int getPointXY(Point p1, Point p2)
{
	return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}
// Determine in which group according to the center of mass
int clusterOfTuple(Tuple means[], Tuple tuple) {
	float dist = getDistXY(means[0], tuple);
	float tmp;
	int label = 0;
	for (int i = 1; i < k; i++) {
		tmp = getDistXY(means[i], tuple);
		if (tmp < dist) { dist = tmp; label = i; }
	}
	return label;
}
// Get the squared error given the group  
float getVar(vector<Tuple> clusters[], Tuple means[]) {
	float var = 0;
	for (int i = 0; i < k; i++)
	{
		vector<Tuple> t = clusters[i];
		for (int j = 0; j < t.size(); j++)
		{
			var += getDistXY(t[j], means[i]);
		}
	}
	return var;

}
// Calculate the center of mass of the given group  
Tuple getMeans(vector<Tuple> cluster) {
	int num = cluster.size();
	double meansX = 0, meansY = 0;
	Tuple t;
	for (int i = 0; i < num; i++)
	{
		meansX += cluster[i].attr1;
		meansY += cluster[i].attr2;
	}
	t.attr1 = meansX / num;
	t.attr2 = meansY / num;
	return t;
}
// Kmeans 
Tuple KMeans(vector<Tuple> tuples) {
	vector<Tuple> clusters[k];
	Tuple means[k];
	Tuple resultMeansf;
	int i = 0;

	for (i = 0; i < k; i++) {
		means[i].attr1 = tuples[i].attr1;
		means[i].attr2 = tuples[i].attr2;
	}
	int lable = 0;

	for (i = 0; i != tuples.size(); ++i) {
		lable = clusterOfTuple(means, tuples[i]);
		clusters[lable].push_back(tuples[i]);
	}

	for (lable = 0; lable < k; lable++) {
		vector<Tuple> t = clusters[lable];
		for (i = 0; i < t.size(); i++)
		{
			//cout << "(" << t[i].attr1 << "," << t[i].attr2 << ")" << "   ";
		}
	}
	float oldVar = -1;
	float newVar = getVar(clusters, means);
	while (abs(newVar - oldVar) >= 1)
	{

		for (i = 0; i < k; i++)
		{
			means[i] = getMeans(clusters[i]);
		}
		oldVar = newVar;
		newVar = getVar(clusters, means);
		for (i = 0; i < k; i++)
		{
			clusters[i].clear();
		}

		for (i = 0; i != tuples.size(); ++i) {
			lable = clusterOfTuple(means, tuples[i]);
			clusters[lable].push_back(tuples[i]);
		}

		for (lable = 0; lable < k; lable++) {
			vector<Tuple> t = clusters[lable];
			for (i = 0; i < t.size(); i++)
			{
				//cout << "(" << t[i].attr1 << "," << t[i].attr2 << ")" << "   ";
			}
		}
	}
	for (int i = 0; i < k; i++)
	{
		resultMeansf.attr1 = means[i].attr1;
		resultMeansf.attr2 = means[i].attr2;
	}
	return resultMeansf;
}
//--------------------------End-----------------

//-------------------------Kalman filtering----------------
// This function filters the boat motion path by using Kalman filter
vector<Point> kalmanFilter(vector<Point> mea, vector<Point> meaRes, double cova, double Q, double R)
{
	if (1 == mea.size())
	{
		Point point;
		//x direction
		Xk_kp1 = double(mea.at(0).x);
		pk_kp1 = cova + Q;
		kg = sqrt((pk_kp1*pk_kp1) / (pk_kp1*pk_kp1 + R*R));
		point.x = Xk_kp1 + kg*(double(mea.at(0).x) - Xk_kp1);
		pk_k = pkp1_kp1 = sqrt((1 - kg)*pk_kp1*pk_kp1);

		//y direction
		Xk_kp1_y = double(mea.at(0).y);
		pk_kp1_y = cova + Q;
		kg_y = sqrt((pk_kp1_y*pk_kp1_y) / (pk_kp1_y*pk_kp1_y + R*R));
		point.y = Xk_kp1_y + kg_y*(double(mea.at(0).y) - Xk_kp1_y);
		pk_k_y = pkp1_kp1_y = sqrt((1 - kg_y)*pk_kp1_y*pk_kp1_y);

		flag_num++;
		//medianKal.push_back(point);
		//est.push_back(point);
		meaRes.push_back(point);
	}
	else if (mea.size() > 1)
	{
		Point point;
		Xk_kp1 = double(meaRes.at(flag_num - 1).x);
		pk_kp1 = pkp1_kp1 + Q;
		kg = pk_kp1 / (R + pk_kp1);
		point.x = Xk_kp1 + kg*(mea.at(flag_num).x - Xk_kp1);
		pk_k = pkp1_kp1 = (1 - kg)*pk_kp1;

		Xk_kp1_y = double(meaRes.at(flag_num - 1).y);
		pk_kp1_y = pkp1_kp1_y + Q;
		kg_y = pk_kp1_y / (R + pk_kp1_y);
		point.y = Xk_kp1_y + kg_y*(mea.at(flag_num).y - Xk_kp1_y);
		pk_k_y = pkp1_kp1_y = (1 - kg_y)*pk_kp1_y;

		flag_num++;
		meaRes.push_back(point);
	}
	return meaRes;
}

#endif