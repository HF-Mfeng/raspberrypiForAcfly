#include "LMZUart.hpp"
#include "ImageFun.hpp"
#include "LMZPID.hpp"

#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>

using namespace cv;
using namespace std;
#define PrintPoint(P,i) std::cout<<#P<<" ["<<##i##<<"] = ("<<##P##[##i##].x<<","<<##P##[##i##].y<<")"<<std::endl

// 定义相机大小
int fd;
const int Laserpin = 1;
const int centerX = 160 ;
const int centerY = 120 ;


ImageFun image(0);

LMZUart myuart("/dev/ttyS0", 57600);



//把一个float拆成4位uint8进行发送；
void Send_Float(float f)
{
	unsigned char s[4];
	unsigned char *p;
	p = (unsigned char *)&f;
	*s = *p;
	*(s+1) = *(p+1);
	*(s+2) = *(p+2);
	*(s+3) = *(p+3);
	serialPutchar (fd, s[0]);
	serialPutchar (fd, s[1]);
	serialPutchar (fd, s[2]);
	serialPutchar (fd, s[3]);
}

//发送三轴速度给飞控
void senddata ( float vx ,float vy ,float vz)
{
	
	myuart.sendMessage("AC");
	serialPutchar (fd, 0xD2);
	serialPutchar (fd, 12);
	Send_Float(vx);
	Send_Float(vy);
	Send_Float(vz);
	serialPutchar (fd, 0);
	serialPutchar (fd, 0);
}

//等待飞控响应（校验速度是否正确）
void receivecall(void)
{
	float SDI_Point[3] = {0};
	unsigned char msg_pack[24] ={0};
	while (true)
	{
		if((char)serialGetchar(fd) == 'A')  //接受包头A
		{
			cout << "receive 'A' "<<endl;
			break;
		}
	}
	for(int i = 0 ; i < 14 ; i++){
		msg_pack[i] = (char)serialGetchar(fd);
	}
	serialFlush(fd);
	SDI_Point[0] = *(float*)&msg_pack[0];
	SDI_Point[1] = *(float*)&msg_pack[5];
	SDI_Point[2] = *(float*)&msg_pack[10];
	cout << SDI_Point[0] << endl;
	cout << SDI_Point[1] << endl;
	cout << SDI_Point[2] << endl;
}

//进行PID调节定点
void fiterPoint(int times = 75, bool draw = true){

	double px = 0.25, py = 0.25 ;
	float vmax = 50 ;
	float vmaxyz = 50 ;
	double pTime = 0.2;
	double startT =  (double)getTickCount();
	int sum = 0;
	Point targetP(-1,-1);
	while (times > 0)
	{
		// if(checkCapture()){
		// 	// Point target = findPoint(srcImage,10,draw);
		// 	Point target;
		// 	double st = (double)getTickCount();
		// 	findRed(srcImage, target, 50, 2000);
		// 	double ALLT = ((double)getTickCount() - st) / getTickFrequency();
		// 	double fps = 1.0 / ALLT ;
		// 	if(target.x > 0 || target.y > 0){
		// 		targetP = target ;
		// 	}

		Point target(-1,-1);
		if(image.readCap()){
			vector<Point> points;
			if(image.findCircle(points))
				target = points[0];
			double st = (double)getTickCount();
			double ALLT = ((double)getTickCount() - st) / getTickFrequency();
			double fps = 1.0 / ALLT ;
			if(target.x > 0 || target.y > 0){
				targetP = target ;
			}

			if(draw){
	//			double dis = getDis();
				string pointInfor = "P(" ;
				pointInfor += to_string(targetP.x) ;
				pointInfor += "," ;
				pointInfor += to_string(targetP.y) ;
				pointInfor += ")" ;
				putText(image.srcImage, pointInfor, /*targetP +*/ Point(5,25), FONT_HERSHEY_COMPLEX, 0.6, Scalar(255,255,255), 1, 8);
	//			pointInfor = "dis = " + to_string(dis) + " cm" ;
	//			putText(srcImage, pointInfor, /*targetP +*/ Point(5,45), FONT_HERSHEY_COMPLEX, 0.6, Scalar(255,255,255), 1, 8);
				pointInfor = "Fps = " + to_string(fps) ;
				putText(image.srcImage, pointInfor, /*targetP +*/ Point(5,65), FONT_HERSHEY_COMPLEX, 0.6, Scalar(255,255,255), 1, 8);				
				imshow("srcImage",image.srcImage);
				// cout << "targetP("<<targetP.x<<","<<targetP.y<<");"<<endl;
				if(waitKey(1) == 'q')
					break;
			}

			double viaTime = ((double)getTickCount() - startT) / getTickFrequency();
			if (viaTime > pTime){  // 进行pid调节	
				// SpeedControl(0,0,0);
				times--;
				if(targetP.x > 0 || targetP.y > 0){  // 点可以使用
					// x 超声波；y，图像的x；z，图像的y；
					float vx = 0, vy = 0 ;
					// double diffx = getDis() - 100;
					float diffx = (float)(centerY - targetP.y);
					float diffy = (float)(centerX - targetP.x);
				//	float diffz = (float)(centerY - targetP.y);
				//	cout << "diffx = " << diffx << ", diffy = " << diffy << ", diffz = " << diffz << endl ;
					if(abs(diffx) < 40 && abs(diffy) < 40 ){  // 不需要调节
						sum++;
						cout << "needn't control --> " << sum << endl ; 
						// pinMode(Laserpin,OUTPUT);  
						// digitalWrite(Laserpin,HIGH); 						
						if(sum > 75)
						{
							cout << "have fixed the point" << endl;
							// digitalWrite(Laserpin,LOW);
							senddata(0,0,0);
							break;
						}
							
					}else{  // 需要调节
						// sum = 0;				
						vx = diffx * px ;
						vy = diffy * py ;
						// digitalWrite(Laserpin,LOW);
						// vz = diffz * pz ;
						// vx = mypidx.PIDControl_byDiff(diffx);
						// vy = mypidyz.PIDControl_byDiff(diffy);
						// vz = mypidyz.PIDControl_byDiff(diffz);

						//限制速度最大值为10cm/秒
						vx = abs(vx) > vmax ? ( vmax*(vx/abs(vx)) ) : vx ;
						vy = abs(vy) > vmaxyz ? ( vmaxyz*(vy/abs(vy)) ) : vy ; 
					//	vz = abs(vz) > vmaxyz ? ( vmaxyz*(vz/abs(vz)) ) : vz ;

						cout << "speedControl" << "-->vx = " << vx << ", vy = " << vy << endl ;
						senddata(vx,vy,0);
					}
				}
				startT =  (double)getTickCount(); // 更新时间
			}
		}
	}	
	senddata(0,0,0);
	destroyAllWindows();
}

//进行延时
void delaySec(int sec){
	for(int i = 0 ; i < sec ; i++){
		cout <<  "\r    wait " << i+1 << " sec"  ;
		usleep(1000*1000);
	}
	cout << endl ;
}

//打开激光2s
void openlaser(void){
	const int LEDpin = 1;
	if(-1==wiringPiSetup())
	{
			cout <<"setup error" << endl;
			exit(-1);
	}
	cout << "open the laser" <<endl;
	pinMode(LEDpin,OUTPUT);  
	digitalWrite(LEDpin,HIGH); 
	usleep(2000*1000);
	digitalWrite(LEDpin,LOW);
}

//等待飞控发送‘1’，树莓派进行下一步操作
void waitmsg(void){
	cout << "waiting for '1' " << endl;
	while (true)
	{
		if((char)serialGetchar(fd) == '1')  //接受包头'1'
		{
			cout << "receive '1' "<<endl;
			break;
		}
	}

}


//进行PID调节定点
void fiterPoint2(int times = 1000, bool draw = true){

	double px = 0.2, py = 0.2 ;
	float vmax = 50 ;
	float vmaxyz = 50 ;
	double pTime = 0.2;
	double startT =  (double)getTickCount();
	int sum = 0;
	Point targetP(-1,-1);

	while (times > 0)
	{
		Point target(-1,-1);
		if(image.readCap()){
			vector<Point> points;
			if(image.findRed(points))
			{
				int posize = points.size();  // 获取点的个数
				if(posize == 1)
				{
					target = points[0];
				}
				if(posize >= 2)
				{
					double distance_0 = fabs(centerX - points[0].x);
					double distance_1 = fabs(centerX - points[1].x);
					if(distance_0 <= distance_1)
					{
						target = points[0];
						cout << "choose point0" << endl;
					}						
					else
					{
						target = points[1];
						cout << "choose point1" << endl;
					}	
				}			
			}				
			double st = (double)getTickCount();
			double ALLT = ((double)getTickCount() - st) / getTickFrequency();
			double fps = 1.0 / ALLT ;


			if(target.x > 0 || target.y > 0){
				targetP = target ;
			}

			if(draw){
	//			double dis = getDis();
				string pointInfor = "P(" ;
				pointInfor += to_string(targetP.x) ;
				pointInfor += "," ;
				pointInfor += to_string(targetP.y) ;
				pointInfor += ")" ;
				putText(image.srcImage, pointInfor, /*targetP +*/ Point(5,25), FONT_HERSHEY_COMPLEX, 0.6, Scalar(255,255,255), 1, 8);
	//			pointInfor = "dis = " + to_string(dis) + " cm" ;
	//			putText(srcImage, pointInfor, /*targetP +*/ Point(5,45), FONT_HERSHEY_COMPLEX, 0.6, Scalar(255,255,255), 1, 8);
				pointInfor = "Fps = " + to_string(fps) ;
				putText(image.srcImage, pointInfor, /*targetP +*/ Point(5,65), FONT_HERSHEY_COMPLEX, 0.6, Scalar(255,255,255), 1, 8);				

				circle(image.srcImage, targetP, 3, Scalar(255,0,0),2);
				imshow("srcImage",image.srcImage);
				// cout << "targetP("<<targetP.x<<","<<targetP.y<<");"<<endl;
				if(waitKey(1) == 'q')
					break;
			}

			double viaTime = ((double)getTickCount() - startT) / getTickFrequency();
			if (viaTime > pTime){  // 进行pid调节	
				// SpeedControl(0,0,0);
				times--;
				if(targetP.x > 0 || targetP.y > 0){  // 点可以使用
					// x 超声波；y，图像的x；z，图像的y；
					float vx = 0, vy = 0 ;
					// double diffx = getDis() - 100;
					float diffx = (float)(centerY - targetP.y);
					float diffy = (float)(centerX - targetP.x);
				//	float diffz = (float)(centerY - targetP.y);
				//	cout << "diffx = " << diffx << ", diffy = " << diffy << ", diffz = " << diffz << endl ;
					if(abs(diffx) < 40 && abs(diffy) < 40 ){  // 不需要调节
						sum++;
						cout << "needn't control --> " << sum << endl ; 
						pinMode(Laserpin,OUTPUT);  
						digitalWrite(Laserpin,HIGH); 						
						if(sum > 10)
						{
							cout << "have fixed the point" << endl;
							digitalWrite(Laserpin,LOW);
							senddata(0,0,0);
							break;
						}
							
					}else{  // 需要调节
						sum = 0;				
						vx = diffx * px ;
						vy = diffy * py ;
						digitalWrite(Laserpin,LOW);
						// vz = diffz * pz ;
						// vx = mypidx.PIDControl_byDiff(diffx);
						// vy = mypidyz.PIDControl_byDiff(diffy);
						// vz = mypidyz.PIDControl_byDiff(diffz);

						//限制速度最大值为10cm/秒
						vx = abs(vx) > vmax ? ( vmax*(vx/abs(vx)) ) : vx ;
						vy = abs(vy) > vmaxyz ? ( vmaxyz*(vy/abs(vy)) ) : vy ; 
					//	vz = abs(vz) > vmaxyz ? ( vmaxyz*(vz/abs(vz)) ) : vz ;

						cout << "speedControl" << "-->vx = " << vx << ", vy = " << vy << endl ;
						senddata(vx,vy,0);
					}
				}
				startT =  (double)getTickCount(); // 更新时间
			}
		}
	}	
	senddata(0,0,0);
	destroyAllWindows();
}

bool draw = true;

int main2(){
	cout << "Version 2.15" << endl ;
	if(-1==wiringPiSetup())
	{
		cout <<"setup error" << endl;
		exit(-1);
	}
	pinMode(Laserpin,OUTPUT); 
	digitalWrite(Laserpin,LOW);
	fd = serialOpen("/dev/ttyS0",57600);
	if(myuart.openUart()){
		// capture.set(CAP_PROP_FRAME_WIDTH, Width);  
		// capture.set(CAP_PROP_FRAME_HEIGHT, Height);
		waitmsg();
		cout << "beginfindcirclepoint" << endl;
		fiterPoint();
		waitmsg();
		fiterPoint2();
		waitmsg();
		fiterPoint2();
		waitmsg();
		fiterPoint2();
		waitmsg();
		fiterPoint2();
		cout << "flyending" << endl;
		myuart.closeUart();
	}else cout << "串口打开失败了" << endl;
    return 0;
}

int main(){
	while(image.wait()){
		if(image.readCap()){

			// 找圆
			vector<Point> points;
			if(image.findCircle(points,200.0,draw))
			{
				if(draw){
					// Point point0 = points[0];
					// PrintPoint(points,0);
					// fiterPoint();
					circle(image.srcImage, points[0], 3, Scalar(0,0,255),2);
					imshow("src",image.srcImage);
				}
			}

			// 找直线和垂直线的交点
			vector<LMZLine> lmzlines;
			vector<Point> vePoint;
			if(image.findLines(lmzlines, 10, draw)){
				if(image.fineVerticalIntersection(vePoint, lmzlines, 10, draw)){
					// cout << "verticalintersection" << endl;
				}
			}

			// 找红色
			vector<Point> redPoints;
			if(image.findRed(redPoints, 100, 5000, draw)){
				// cout << "red" << endl;
			}
			
			if(draw){
				string str = "fps = " + to_string((int)image.getFps()) ;
				putText(image.srcImage, str, Point(10,10), 1, 0.8, Scalar(0,0,255));
				imshow("srcImage",image.srcImage);
			}
			// cout <<"fps = "<< image.getFps() <<endl;
		}
		
	}
		// if(checkCapture()){
		// 	double fps,t ;
		// 	t =  getTickCount();
		// 	Point target = findCircle(srcImage,200, true);
		// 	circle(srcImage, target, 3, Scalar(255,0,0),2);
			
		// 	t = ((double)getTickCount() - t) / getTickFrequency();
		// 	fps = 1.0 / t ;
		// 	String str = "fps = " + to_string((int)fps);
		// 	putText(srcImage, str, target + Point(3,3), FONT_HERSHEY_COMPLEX, 0.6, Scalar(255,0,0), 1, 8);	
		// 	imshow("findCircle",srcImage);
		// }
		// }
	return 0;
}
//      g++ -Wall UartTest.cpp -lwiringPi -o wall
//		 cd Desktop/FLY_2021_C/


