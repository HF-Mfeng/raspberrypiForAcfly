#ifndef IMAGEFUN_HPP
#define IMAGEFUN_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <string>

//#define NEEDDEBUG
#ifdef NEEDDEBUG
#include <iostream>
#define Debug(x) std::cout<<"debug-->"<<x<<std::endl
#else
#define Debug(x) ;
#endif

// void initMyLine(cv::Point& point, double k) {
//     if (k == 0) {
//         k = 0.0000001;
//     }
//     // this->k = k;
//     // this->b = (point.y - k * point.x);
//     this->pointA = point;
//     int x_temp = 2 * point.x;
//     int y_temp = (int)(k * x_temp + b);
//     cv::Point temp(x_temp, y_temp);
//     this->pointB = temp;
// }
// // ��������ֱ��֮��ĽǶ�
// static double countAngle(LMZLine& line1, LMZLine& line2) {
//     double angle1 = atan(line1.k); // ������
//     double angle2 = atan(line2.k); // ������
//     return (angle2 - angle1) * 180.0 / LMZ_PI;
// }



#define LMZ_PI 3.14159265358979323846
class LMZLine {
public:
	// ����
	cv::Point pointA, pointB;

	// ����
	LMZLine() :pointA(cv::Point(0, 0)), pointB(cv::Point(0, 0)) {
	}
	LMZLine(const LMZLine& line) {
		this->pointA = line.pointA;
		this->pointB = line.pointB;
	}
	LMZLine(cv::Point PointA, cv::Point PointB) :pointA(PointA), pointB(PointB) {
	}


	// ����ֱ����x��ĽǶȣ���Χ�� 0��180��
	double countAngle() {
		double angle = atan2((this->pointA.y - this->pointB.y), (this->pointA.x - this->pointB.x)); // �Ƕ�
		angle = angle * 180 / LMZ_PI;
		if (angle < 0)
			angle += 180; // �޷��� 0-180 ��
		return angle;
	}
	/*
	 * �����ֱ�ߵ�б��
	 * ���б���ر�󣬾ͻ᷵�� DBL_MAX
	 */
	double countK() {
		if (this->pointA.x == this->pointB.x) {
			return DBL_MAX;       // �������ֵ
		}
		return ((double)((double)this->pointA.y - this->pointB.y)) / ((double)this->pointA.x - this->pointB.x);
	}

	/*
	 * �����A��B֮���ֱ�߷��̵�б����ؾ�
	*/
	static bool countKB(cv::Point& pointA, cv::Point& pointB, double& Lk, double& Lb) {
		if (pointA.x == pointB.x) {
			Lk = DBL_MAX;
			Lb = 0;
			return false;
		}
		Lk = ((double)((double)pointA.y - pointB.y)) / ((double)pointA.x - pointB.x);
		Lb = (double)pointB.y - pointB.x * Lk;
		return true;
	}

	// ���ء�=�������
	LMZLine& operator=(const LMZLine& line) {
		// ������Ȼ��ֵ
		if (this != &line) {
			this->pointA = line.pointA;
			this->pointB = line.pointB;
		}
		return *this;
	}


	// �ж�����ֱ���Ƿ񼸺���ֱ������ֱ�߽Ƕȷ�Χ�� 90��� maxAngle��
	static bool isVertical(LMZLine& line1, LMZLine& line2, double maxAngle = 5) {
		double angle1 = line1.countAngle();
		double angle2 = line2.countAngle();
		if (abs(abs(angle1 - angle2)-90) < abs(maxAngle))
			return true;
		return false;
	}
	// �ж��Ƿ�ƽ�У��Ƕȷ�Χ�� ��5��
	static bool isParallel(LMZLine& line1, LMZLine& line2, double maxAngle = 5) {
		double angle1 = line1.countAngle();
		double angle2 = line2.countAngle();
		if (angle1 == angle2)
			return false;

		if (abs(angle1 - angle2) < abs(maxAngle) || abs(angle1 - angle2) > 180 - abs(maxAngle))
			return true;
		//double  bigAngle = angle1 > angle2 ? angle1 : angle2;
		//double  smallAngle = angle1 < angle2 ? angle1 : angle2;
		//if (abs(180 - bigAngle + smallAngle) < abs(maxAngle))
		//	return true;
		return false;
	}

	// �õ�����ֱ��֮��Ľ���
	// �������ֱ�߼���ƽ�У��ͷ���(INT32_MAX,INT32_MAX)
	// maxAngle������ֱ�ߵļн�Ϊ���پ���Ϊ�Ǽ���ƽ�е�
	static cv::Point get_Intersection(cv::Point& line1_point1, cv::Point& line1_point2, cv::Point& line2_point1, cv::Point& line2_point2, double maxAngle = 5) {
		cv::Point intersection(INT32_MAX, INT32_MAX);   // int����Сֵ
		// �������ֱ�߼���ƽ��
		LMZLine line1(line1_point1, line1_point2);
		LMZLine line2(line2_point1, line2_point2);
		//if (abs(line1.countAngle() - line2.countAngle()) < maxAngle)     // ����ֱ�߼���ƽ��
		if (isParallel(line1, line2, maxAngle))   // ����ֱ�߼���ƽ��
			return intersection;
		double k1, k2, b1, b2; // ֱ��1��б�ʺͽؾֱ࣬��2��б�ʺͽؾ�
		LMZLine::countKB(line1_point1, line1_point2, k1, b1);
		LMZLine::countKB(line2_point1, line2_point2, k2, b2);
		double x, y;
		x = (b2 - b1) / (k1 - k2);
		y = (k2 * b1 - b2 * k1) / (k2 - k1);
		intersection.x = (int)x;
		intersection.y = (int)y;
		return intersection;
	}
	static cv::Point get_Intersection(LMZLine& line1, LMZLine& line2, double maxAngle = 5) {
		return get_Intersection(line1.pointA, line1.pointB, line2.pointA, line2.pointB, maxAngle);
	}
};

class ImageFun
{
private:

public:
	ImageFun(int cap = 0 , unsigned int width = 320, unsigned int height = 240) {
		while (!(this->capture.open(cap))) std::cout << "can't open capture " << cap << std::endl;       // ������ͷ
		this->captureIndex = cap;      		// ����cap
		this->Height = height;					// ������
		this->Width = width;					// ����߶�
		this->startTick = 0;
		this->fps = 0;
		this->centerX = this->Width / 2;
		this->centerY = this->Height / 2;
	}
	~ImageFun() {
		
	}

	// ����
	cv::VideoCapture capture;
	int captureIndex;
	cv::Mat srcImage;
	unsigned int Width, Height;
	double startTick;
	double fps;
	unsigned int centerX, centerY;

	enum GrayColor {
		black = 0,
		white = 255,
	};

	bool readCap(unsigned int width = 0,unsigned int height = 0) {
		bool isCap = this->capture.read(this->srcImage);
		// if (width > 0 && height > 0 && ( width != this->Width || height != this->Height ) ) {
		// 	this->Width = width;
		// 	this->Height = height;
		// 	this->centerX = this->Width / 2;
		// 	this->centerY = this->Height / 2;
		// }
		
		if( width > 0 && width != this->Width ){
			this->Width = width;
			this->centerX = this->Width / 2;
		}
		if( height > 0 && height != this->Height ){
			this->Height = height;
			this->centerY = this->Height / 2;
		}

		resize(this->srcImage, this->srcImage, cv::Size(this->Width, this->Height));
		return isCap;
	}

	double getFps(double startT = -1) {
		if (startT < 0) startT = this->startTick;
		double allT = ((double)cv::getTickCount() - startT) / cv::getTickFrequency();
		this->fps = 1.0 / allT;
		this->startTick = cv::getTickCount();  // ����ʱ��
		return this->fps;
	}
	bool wait(int ms = 1) {
		return ( cv::waitKey(ms) != 27 );
	}

	// ͼ����ĺ���

	/*
	 * ������
	 *		����ͼ�񣬵㼯���������1���������2���Ƿ�ͼ
	 * 	����ͼ��
	 *  �㼯���ҵ����ٸ�Բ�ģ������ڴ�
	 *  �������������������õ����������������1���������2֮��Ĳ���Ϊ����Ҫ�ĺ�ɫ
	 *  �Ƿ�ͼ���Ƿ�ͼ
	 * ����ֵ��
	 * 	bool������ҵ��˷���true�����򷵻�false
	*/
	bool findRed(std::vector<cv::Point>& red_centers, double area1 = 100, double area2 = 5000, bool draw = false) {
		cv::Mat tempImage, thisFunDstImage;
		this->srcImage.copyTo(tempImage);
		if (draw) this->srcImage.copyTo(thisFunDstImage);
		// ͼ������
		cv::cvtColor(tempImage, tempImage, cv::COLOR_BGR2HSV);
		cv::Scalar lower(0, 100, 100);
		cv::Scalar upper(10, 255, 255);
		cv::Scalar lower2(156, 100, 100);
		cv::Scalar upper2(180, 255, 255);
		cv::Mat dst, dst2, dst1;
		cv::inRange(tempImage, lower, upper, dst1);
		cv::inRange(tempImage, lower2, upper2, dst2);
		dst = dst1 | dst2;													// ���������
		dst = ~dst;															// ȡ�����õ����Ǻڰ�ͼƬ

		// ��������
		cv::Mat kenerl = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
		cv::morphologyEx(dst, dst, cv::MORPH_CLOSE, kenerl);						// ������
		cv::morphologyEx(dst, dst, cv::MORPH_OPEN, kenerl);							// ������
		dst = ~dst;

		if (draw) cv::imshow("findRed", dst);
		bool isRed = false;
		// ���Ƿ����ҵ���ɫ����Ӧdst�ϵĺ�ɫ��ʹ�����������Ѱ��
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(dst, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
		if (draw) cv::drawContours(thisFunDstImage, contours, -1, cv::Scalar(255, 0, 0)); // ������������
		// ���������������ҵ����ں������������
		for (size_t i = 0; i < contours.size(); i++) {
			cv::RotatedRect rect = cv::minAreaRect(contours[i]);
			double contour_area = cv::contourArea(contours[i]);
		//	std::cout << "contour_area[" << i << "] = " << contour_area << std::endl;
			// ����������Ҫ���
			if (contour_area >= area1 && contour_area <= area2) {
				cv::Point centerPoint(rect.center.x, rect.center.y);   // Բ��
				if (draw) cv::circle(thisFunDstImage, centerPoint, 3, cv::Scalar(0, 255, 255), 3);
				isRed = true;
				red_centers.push_back(centerPoint);
			}
		}
		if (draw) cv::imshow("findRedDst", thisFunDstImage);
		return isRed;
	}

	// ͬ�ϣ��㼯��ɵ�һ�ĵ����
	bool findRed(cv::Point& red_center, double area1 = 100, double area2 = 5000, bool draw = false) {
		cv::Mat tempImage, thisFunDstImage;
		this->srcImage.copyTo(tempImage);
		if (draw) this->srcImage.copyTo(thisFunDstImage);
		// ͼ������
		cv::cvtColor(tempImage, tempImage, cv::COLOR_BGR2GRAY);
		cv::Scalar lower(0, 100, 100);
		cv::Scalar upper(10, 255, 255);
		cv::Scalar lower2(156, 100, 100);
		cv::Scalar upper2(180, 255, 255);
		cv::Mat dst, dst2, dst1;
		cv::inRange(tempImage, lower, upper, dst1);
		cv::inRange(tempImage, lower2, upper2, dst2);
		dst = dst1 | dst2;													// ���������
		dst = ~dst;															// ȡ�����õ����Ǻڰ�ͼƬ

		// ��������
		cv::Mat kenerl = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
		cv::morphologyEx(dst, dst, cv::MORPH_CLOSE, kenerl);						// ������
		cv::morphologyEx(dst, dst, cv::MORPH_OPEN, kenerl);							// ������
		dst = ~dst;

		if (draw) cv::imshow("findRed", dst);
		bool isRed = false;
		// ���Ƿ����ҵ���ɫ����Ӧdst�ϵĺ�ɫ��ʹ�����������Ѱ��
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(dst, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
		if (draw) cv::drawContours(thisFunDstImage, contours, -1, cv::Scalar(255, 0, 0)); // ������������
		// ���������������ҵ����ں������������
		double maxArea = 0;
		for (size_t i = 0; i < contours.size(); i++) {
			cv::RotatedRect rect = cv::minAreaRect(contours[i]);
			double contour_area = cv::contourArea(contours[i]);
			// ����������Ҫ���
			if (contour_area >= area1 && contour_area <= area2) {
				cv::Point centerPoint(rect.center.x, rect.center.y);   // Բ��
				if (draw) cv::circle(thisFunDstImage, centerPoint, 3, cv::Scalar(0, 255, 255), 3);
				if (contour_area > maxArea) {  // �ҵ�����ģ����¸�ֵ
					red_center = centerPoint;
					maxArea = contour_area;
				}
				isRed = true;
			}
		}
		if (draw) cv::imshow("findRedDst", thisFunDstImage);
		return isRed;
	}

	/*
	 * ������
	 *	srcImage������ͼ��
	 * 	points���ҵ�ֱ�ߵ����н��㱣������
	 *  angle_threshold��Ԥ��λ
	 *  draw���Ƿ�ͼ
	 *  outLines������λ�����ҵ���ֱ�߶���������
	 * ����ֵ��
	 * 	�����ҵ����ٸ���
	 *
	*/
	unsigned int findLinePoint(std::vector<cv::Point>& points, double angle_threshold = 10, bool draw = false, std::vector<LMZLine>* outlines = NULL) {
		std::vector<LMZLine> lines;
		cv::Mat temp, dst;
		this->srcImage.copyTo(temp);
		if (draw) this->srcImage.copyTo(dst);

		cv::cvtColor(temp, temp, cv::COLOR_BGR2GRAY);
		cv::Mat kenerl = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
		cv::morphologyEx(temp, temp, cv::MORPH_CLOSE, kenerl);							// �ȱ�����
		cv::morphologyEx(temp, temp, cv::MORPH_OPEN, kenerl);							// �ٿ�����
		cv::Canny(temp, temp, 300, 100);

		if (draw) cv::imshow("canny", temp);

		std::vector<cv::Vec4i> plines;
		HoughLinesP(temp, plines, 1, CV_PI / 180, 30, 30, 120);
		for (auto aline : plines) {
			cv::Point pa(aline[0], aline[1]), pb(aline[2], aline[3]);
			if (draw) line(dst, pa, pb, cv::Scalar(255, 0, 0), 2);
			LMZLine tempLine(pa, pb);
			lines.push_back(tempLine);
		}
		if (lines.empty()) return 0;
		// if( outlines != NULL) *outlines = lines;     // ��ֵ

		unsigned int pointNum = 0;
		if (lines.size() > 1) // ��ֹһ��ֱ��
		{
			for (size_t i = 0; i < lines.size(); i++) {
				for (size_t j = i + 1; j < lines.size(); j++) {
					cv::Point temp_p = LMZLine::get_Intersection(lines[i], lines[j]);
					points.push_back(temp_p);
					pointNum++;
					if (draw) cv::circle(dst, temp_p, 2, cv::Scalar(0, 255, 255), 2);
				}
			}
		}
		else return 0;
		if (draw) {
			std::string str("There are ");
			str += std::to_string(lines.size());
			str += " Lines.";
			cv::putText(dst, str, cv::Point(10, 50), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(127, 0, 128), 1, 8);
			str = "There are ";
			str += std::to_string(pointNum);
			str += " Points.";
			cv::putText(dst, str, cv::Point(10, 80), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(127, 0, 128), 1, 8);
			imshow("dst", dst);
		}
		return pointNum;
	}

	// Ѱ��ֱ�߲�������㣬�����ҵ�������
	unsigned int findLinePoint(cv::Point& point, double angle_threshold = 10, bool draw = false, std::vector<LMZLine>* outlines = NULL) {
		std::vector<LMZLine> lines;
		cv::Mat temp, dst;
		this->srcImage.copyTo(temp);
		if (draw) this->srcImage.copyTo(dst);

		cv::cvtColor(temp, temp, cv::COLOR_BGR2GRAY);
		cv::Mat kenerl = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
		cv::morphologyEx(temp, temp, cv::MORPH_CLOSE, kenerl);							// �ȱ�����
		cv::morphologyEx(temp, temp, cv::MORPH_OPEN, kenerl);							// �ٿ�����
		cv::Canny(temp, temp, 300, 100);

		if (draw) cv::imshow("canny", temp);

		std::vector<cv::Vec4i> plines;
		HoughLinesP(temp, plines, 1, CV_PI / 180, 30, 30, 120);
		for (auto aline : plines) {
			cv::Point pa(aline[0], aline[1]), pb(aline[2], aline[3]);
			if (draw) line(dst, pa, pb, cv::Scalar(255, 0, 0), 2);
			LMZLine tempLine(pa, pb);
			lines.push_back(tempLine);
		}
		if (lines.empty()) return 0;
		// if( outlines != NULL) *outlines = lines;     // ��ֵ

		unsigned int pointNum = 0;
		std::vector<cv::Point> points;
		if (lines.size() > 1) // ��ֹһ��ֱ��
		{
			for (size_t i = 0; i < lines.size(); i++) {
				for (size_t j = i + 1; j < lines.size(); j++) {
					cv::Point temp_p = LMZLine::get_Intersection(lines[i], lines[j]);
					points.push_back(temp_p);
					pointNum++;
					if (draw) cv::circle(dst, temp_p, 2, cv::Scalar(0, 255, 255), 2);
				}
			}
		}
		else return 0;
		point = points[0];
		if (draw) {
			std::string str("There are ");
			str += std::to_string(lines.size());
			str += " Lines.";
			cv::putText(dst, str, cv::Point(10, 50), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(127, 0, 128), 1, 8);
			str = "There are ";
			str += std::to_string(pointNum);
			str += " Points.";
			cv::putText(dst, str, cv::Point(10, 80), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(127, 0, 128), 1, 8);
			imshow("dst", dst);
		}
		return pointNum;
	}

	/*
	* ������params2��Խ��Բ�ͻ�Խ������ԽС�ҵ���αԲԽ��
	* ����ֵ���ҵ����ٸ�Բ
	*/
	unsigned int findCircle(std::vector<cv::Point> &centers, double params2 = 200, bool draw = false) {
		cv::Mat temp, dst;
		this->srcImage.copyTo(temp);
		if (draw) this->srcImage.copyTo(dst);

		cv::cvtColor(temp, temp, cv::COLOR_BGR2GRAY);
		cv::GaussianBlur(temp, temp, cv::Size(9,9),2,2);
		
		// �ҵ����е�Բ
		std::vector<cv::Vec3f> pcircles;
		cv::HoughCircles(temp, pcircles, cv::HOUGH_GRADIENT, 2, temp.rows / 4, 200, params2);
		unsigned int centerNum = 0;
		for ( auto acircle : pcircles){
			cv::Point acenter(acircle[0], acircle[1]);
			int radix = acircle[2];
			if (draw) {
				cv::circle(dst, acenter, radix, cv::Scalar(0, 255, 0), 2);
				cv::circle(dst, acenter, 2, cv::Scalar(255, 0, 0), 2);
				std::string str = "Point(" + std::to_string(acenter.x) + "," + std::to_string(acenter.y) + ")";
				cv::putText(dst, str, acenter + cv::Point(3, 3), 3, 0.65,cv::Scalar(255, 0, 0));
			}
			centers.push_back(acenter);
			centerNum++;
		}
		if (draw) cv::imshow("findCircle", dst);
		//std::cout << centers.size() << std::endl;
		//std::cout << centerNum << std::endl;
		return centerNum;
	}

	/*
	 * ֱ�߼�⣺���ض�����ֱ��
	 *		�õ���ֱ����ϣ�����������ƽ�е�ֱ����Ϊ��һ��ֱ��
	 *		maxAngle:����ֱ��ֱ�ӵĽǶ���maxAngle��Ϊ��ƽ�е�
	*/
	unsigned int findLines(std::vector<LMZLine> &allLines,double maxAngle = 5, bool draw = false) {
		cv::Mat temp, dst;
		this->srcImage.copyTo(temp);
		if (draw)this->srcImage.copyTo(dst);

		cv::cvtColor(temp, temp, cv::COLOR_BGR2GRAY);
		cv::Mat kenerl = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
		cv::morphologyEx(temp, temp, cv::MORPH_CLOSE, kenerl);
		cv::morphologyEx(temp, temp, cv::MORPH_OPEN, kenerl);
		cv::Canny(temp, temp, 300, 100);

		if (draw) cv::imshow("cannyFindLines",temp);


		std::vector<LMZLine> lines;
		std::vector<cv::Vec4i> plines;
		HoughLinesP(temp, plines, 1, CV_PI / 180, 30, 60, 120);
		if (plines.empty()) return 0;
		for (auto aline : plines) {
			cv::Point pa(aline[0], aline[1]), pb(aline[2], aline[3]);
			if (draw) { cv::Scalar color(255, 255, 0); line(dst, pa, pb, color, 1); }
			LMZLine tempLine(pa, pb);
			lines.push_back(tempLine);
		}
		if (lines.empty()) return 0;  // �����ⲻ��ֱ�ߣ���ֱ�ӷ���0

		/*
		* ���ֱ�ߣ����Ƕ������ֱ�߿�����һ��ֱ�ߣ��������
		*/
		for (size_t i = 0; i < lines.size() - 1; i++) {
			// �������еĵ�
			std::vector<cv::Point> points_all;
			points_all.push_back(lines[i].pointA);
			points_all.push_back(lines[i].pointB);
			// Ѱ�ҽ���������ƽ�е�ֱ��
			for (size_t j = i + 1; j < lines.size(); j++) {
				if (LMZLine::isParallel(lines[i], lines[j])) {
					points_all.push_back(lines[j].pointA);
					points_all.push_back(lines[j].pointB);
					lines.erase(lines.begin() + j);
					j--;
				}
			}
			// û���ҵ�����ƽ�е�ֱ��
			if (points_all.size() < 3) {
				allLines.push_back(lines[i]);
			}
			else {
				cv::Vec4f line_param;
				fitLine(points_all, line_param, cv::DIST_L2, 0, 1e-2, 1e-2);
				// ��ȡ��бʽ�ĵ��б��
				cv::Point point0;
				point0.x = line_param[2];
				point0.y = line_param[3];
				double k = line_param[1] / line_param[0];
				cv::Point point1, point2;
				point1.x = 0;
				point1.y = k * (0 - (double)point0.x) + point0.y;
				point2.x = Width;
				point2.y = k * (Width - (double)point0.x) + point0.y;
				if (draw) line(dst, point1, point2, cv::Scalar(0, 0, 255), 2);
				LMZLine tempLMZLine(point1, point2);
				allLines.push_back(tempLMZLine);
			}
		}
		int num = (int)allLines.size();
		if (draw) {
			std::string str = "Have " + std::to_string(num) + " lines.";
			cv::putText(dst, str, cv::Point(10, 10), 1, 0.8, cv::Scalar(0, 255, 255));
			cv::imshow("findAllLines", dst);
		}
		return num;
	}
	// ����Ƕȷ�Χ��maxAngle
	unsigned int fineVerticalIntersection(std::vector<cv::Point>& points, std::vector<LMZLine>& lines,double maxAngle = 10, bool draw = false) {
		if (lines.size() < 2) return 0;
		unsigned int pointNum = 0;
		cv::Mat dst;
		if (draw) this->srcImage.copyTo(dst);
		for (size_t i = 0; i < lines.size(); i++) {
			for (size_t j = i + 1; j < lines.size(); j++) {
				if (LMZLine::isVertical(lines[i], lines[j], maxAngle)) {        // ���������ֱ�Ļ�
					cv::Point temp_p = LMZLine::get_Intersection(lines[i], lines[j]);
					points.push_back(temp_p);
					pointNum++;
					if (draw) {
						cv::circle(dst, temp_p, 5, cv::Scalar(0, 255, 255), 3);
						std::string str = "P (" + std::to_string(temp_p.x) + ',' + std::to_string(temp_p.y) + ')';
						cv::putText(dst, str, temp_p + cv::Point(5, 5), 1, 0.8, cv::Scalar(255, 0, 0));
					}
				}
			}
			if (draw) cv::line(dst, lines[i].pointA, lines[i].pointB, cv::Scalar(255, 0, 0), 2);
		}
		if (draw) {
			std::string str = "Have " + std::to_string(pointNum) + " points.";
			cv::putText(dst, str, cv::Point(10, 10), 1, 0.8, cv::Scalar(255, 0, 0));
			cv::imshow("verticalIntersection", dst);
		}
		return pointNum;
	}
	unsigned int findIntersection(std::vector<cv::Point> &points, std::vector<LMZLine>& lines, bool draw = false) {
		if (lines.size() < 2) return 0;
		unsigned int pointNum = 0;
		cv::Mat dst;
		if (draw) this->srcImage.copyTo(dst);
		for (size_t i = 0; i < lines.size(); i++) {
			for (size_t j = i + 1; j < lines.size(); j++) {
				if (!(LMZLine::isParallel(lines[i], lines[j]))) {
					cv::Point temp_p = LMZLine::get_Intersection(lines[i], lines[j]);
					points.push_back(temp_p);
					pointNum++;
					if (draw) cv::circle(dst, temp_p, 2, cv::Scalar(0, 255, 255), 2);
				}
			}
			if (draw) cv::line(dst, lines[i].pointA, lines[i].pointB, cv::Scalar(255, 0, 0), 2);
		}
		if (draw) cv::imshow("Intersection",dst);
		return pointNum;
	}

}; // �������

#endif
