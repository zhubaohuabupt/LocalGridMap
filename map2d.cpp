#include"map2d.h"
namespace map2d

{
	Cemara::Cemara(){
		bf=47.9;
		fx=435.211;
		fy=435.211;
		cx=367.141;
		cy=252.2;
	}
	GradMap::GradMap(cv::Mat& depthpic,int MapSize,int GridSize)
	{
	this->depthpic=depthpic;
	this->MapSize=MapSize;
	this->GridSize=GridSize;
	depthpicHeight=depthpic.rows;
	depthpicWidth=depthpic.cols;
	mapYmax=3;
	mapYmin=0;
	DoNotScan=20*MapSize/500;
	filter=false;
	}

void GradMap::InitgridMap(cv::Mat&GridMap,int Mapsize,int gridSize)
	{
	//根据地图实际大小和栅格尺寸设定栅格地图尺寸
	int GridMaprow=Mapsize/gridSize*3;
	int GridMapcol=Mapsize/gridSize*3;
	while(Mapsize%gridSize!=0)
		Mapsize--;
	GridMap=Mat::zeros(GridMaprow,GridMapcol,CV_8UC1);
	 MapCenter=Point(GridMap.rows/2,GridMap.cols/2);
	int CenterY=MapCenter.y;
	int CenterX=MapCenter.x;
	int z=0;
	int mapz=0,mapx=0;
	int width=depthpic.cols;
   GridMap.at<uchar>(CenterY,CenterX)=255;
	while(z<=Mapsize)//CM
	{
		 z+=gridSize;
		 for(int col=0;col<width;col++)
		 {
		     float x=z*(col-cam.cx)/cam.fx;
			 mapx=x/gridSize;
			 mapz=z/gridSize;
			
			 GridMap.at<uchar>(CenterY-mapz,CenterX+mapx)=100;
			 localGridMapCoordinate.push_back(Point(CenterX+mapx,CenterY-mapz));
		 }
		 if(z==Mapsize)
		 {      int lastx=0;
			  for(int col=0;col<width;col++)
				 {
					 float x=z*(col-cam.cx)/cam.fx;
					 mapx=x/gridSize;
					 mapz=z/gridSize;
					 
					 if(lastx!=mapx)
					  farestline.push_back(Point(CenterX+mapx,CenterY-mapz)); 
					 lastx=mapx;
		 
				 }
			 
		 }
			 
	}
  }
   void GradMap::Get3DPointFromDepthpic()
	  { 
			for(int row=0;row<depthpicHeight;row++ )
			 for(int col=0;col<depthpicWidth;col++)
			 {
				 
				float z=depthpic.at<float>(row,col);
				if(z>10)continue;
				 float x=z*(col-cam.cx)/cam.fx;
				 float y=-z*(row-cam.cx)/cam.fx;
				 if(y>mapYmin&&y<mapYmax)
				 {
					map2d::Point3f P3D(x,y,z);
					piont3d_cam.push_back(P3D);
				 }
			
			 }  
	  }
void GradMap::GetGridMapOccupy(cv::Mat&GridMap)
		{			InitgridMap(GridMap,MapSize,GridSize);
			       Get3DPointFromDepthpic();//获取三维点
				   Mat OccupyInEveryGrid=Mat::zeros(500,500,CV_8UC1);//统计每个栅格里投影的像素点数，大于一定值则认为次栅格是障碍。标记50
  				 for(vector<map2d::Point3f>::iterator it=piont3d_cam.begin();it!=piont3d_cam.end();it++)
					 {			
							   int ratio=100;//把三维点 单位m 转化成栅格 单位cm
							   int mapx=ratio*(*it).x/GridSize;
							   int  mapz=ratio*(*it).z/GridSize;
							   if(MapCenter.y-mapz>GridMap.rows||MapCenter.y-mapz<0||MapCenter.x+mapx>GridMap.cols||MapCenter.x+mapx<0) continue;
							++OccupyInEveryGrid.at<uchar>(MapCenter.y-mapz,MapCenter.x+mapx);	  		 						
					}
	

				for(auto localmapPoint:localGridMapCoordinate){
					 if(OccupyInEveryGrid.at<uchar>(localmapPoint.y,localmapPoint.x)>15)
						 OccupyInEveryGrid.at<uchar>(localmapPoint.y,localmapPoint.x)=50;	 
				  }
				//从相机位置扫描，顺序：从左至右，由近及远直至遇到障碍为止。					
				for(int i=0;i<farestline.size();i++) 
				{ 
			
						Point dest=farestline[i];
						float k=0;
						if(dest.x==MapCenter.x)
							k==0;
						else
							k=(float)(dest.y-MapCenter.y)/(dest.x-MapCenter.x);
						float b=dest.y-k*dest.x;
						float x=0;
						bool unknown=false;
				
						for(int y=MapCenter.y;y>=dest.y;y--)
						 {
							if(k==0)
								 x= MapCenter.x;
							else
								 x=(y-b)/k;
								GridMap.at<uchar>(y,x)=255;	
							if( OccupyInEveryGrid.at<uchar>(y,x)==50||i<DoNotScan)	//最左侧认为是障碍，因为深度图不准。			
										break;								
						 }											
			   }
	  //   if(filter)//是否要对栅格图进行滤波
		//	FilterGridMap(GridMap);
		
		}
void GradMap::FilterGridMap(cv::Mat&GridMap)
{

	for(auto localmapPoint:localGridMapCoordinate)
		  {
					 Point pt=localmapPoint;
					 if(GridMap.at<uchar>(pt.y,pt.x)==255)
					 {
						   int row_begin=pt.y-1;
						   int row_end=pt.y+1;
						   int col_begin=pt.x-1;
						   int col_end=pt.x+1;
						   int cnt=0;
						   for(int j=row_begin;j<=row_end;j++)
						   for(int i=col_begin;i<=col_end;i++) {
								   if(GridMap.at<uchar>(j,i)==255)
									   cnt++;
							   }
						  if(cnt<=4)
							  GridMap.at<uchar>(pt.y,pt.x)=100;
					 }
				}   		 
		   }

}
