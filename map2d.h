#include <opencv2/opencv.hpp>
#include<iostream>
using namespace std;
using namespace cv;
namespace map2d
{
	struct Point3f
	{
		Point3f(float x,float y,float z):x(x),y(y),z(z){}
		float x,y,z;
	};
	class Cemara
	{
	public:
		Cemara();
		
	 float bf;
	 float fx;
	 float fy;
	 float cx;
	 float cy;
	};
	class GradMap{

	public:
	GradMap(cv::Mat& depthpic,int MapSize,int GridSize);
	void InitgridMap(cv::Mat&GridMap,int Mapsize,int gradSize);
	void Get3DPointFromDepthpic();
	void GetGridMapOccupy(cv::Mat&GridMap);
	void FilterGridMap(cv::Mat&GridMap);
	
	Cemara cam;
	cv::Mat depthpic;
	cv::Point MapCenter;
	vector<map2d::Point3f> piont3d_cam;
	vector<cv::Point> localGridMapCoordinate;
	vector<cv::Point> farestline;

	float mapYmax;//����Դ�߶�
	float mapYmin;
	int depthpicHeight;
	int depthpicWidth;
	//ע��  MapSize/GridSizeҪ��֤<300
	int MapSize;//դ���ͼ�Ĵ�С,��λcm
	int GridSize;//ÿһ��դ��Ĵ�С,��λcm
	int DoNotScan;//��������ͼ��Ч����դ���ͼ����Ϊ��Ч����
	bool filter;//�Ƿ�Ҫ��դ���ͼ�˲�
	};
	
	
}