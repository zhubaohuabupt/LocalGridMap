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

	float mapYmax;//数据源高度
	float mapYmin;
	int depthpicHeight;
	int depthpicWidth;
	//注意  MapSize/GridSize要保证<300
	int MapSize;//栅格地图的大小,单位cm
	int GridSize;//每一个栅格的大小,单位cm
	int DoNotScan;//最左侧深度图无效，在栅格地图上视为无效区域。
	bool filter;//是否要对栅格地图滤波
	};
	
	
}