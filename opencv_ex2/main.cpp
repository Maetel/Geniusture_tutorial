#define MINE
#ifndef MINE
#define DO_OPENCV
#endif

#ifdef MINE
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/video/tracking.hpp"
#include <Eigen/Dense>
#include <deque>
#include <numeric>

using namespace std;

using Point = cv::Point2f;
using PointVec = std::vector<Point>;
using IVec = std::vector<int>;
using DVec = std::vector<double>;

enum ROI_ORDER { X_BEGIN = 0, X_END, Y_BEGIN, Y_END, NUM_VARS };

void ERR(string content) {
	std::cout << content << std::endl;
}

void get_ROI_by_ratio(int w, int h, double x_ratio, double y_ratio, IVec& out_vector) {
	auto x_off = (w / 2.0)*x_ratio;
	auto y_off = (h / 2.0)*y_ratio;

	IVec result(NUM_VARS);
	result[X_BEGIN] = x_off;
	result[X_END] = w - 1 - x_off;
	result[Y_BEGIN] = y_off;
	result[Y_END] = h - 1 - y_off;

	out_vector = result;
}

void get_points_in_ROI(int x_pnt, int y_pnt, const IVec& ROI, PointVec& out_points) {
	if (ROI.size() != NUM_VARS) {
		ERR("ROI not initialized");
		return;
	}

	auto x_o = ROI[X_BEGIN];
	auto y_o = ROI[Y_BEGIN];
	auto x_interval = (ROI[X_END] - ROI[X_BEGIN]) / (double)(x_pnt - 1);
	auto y_interval = (ROI[Y_END] - ROI[Y_BEGIN]) / (double)(y_pnt - 1);

	out_points.clear();
	out_points.resize(x_pnt*y_pnt);
	//result.reserve((size_t)x_pnt*y_pnt);

	int _idx = 0;
	for (int y = 0; y < y_pnt; y++) {
		for (int x = 0; x < x_pnt; x++) {
			//Point pnt(x_o + x * x_interval, y_o + y * y_interval);
			out_points[_idx++] = Point(x_o + x * x_interval, y_o + y * y_interval);
		}
	}

}


inline double dist_2pts(const Point& p1, const Point& p2) {

	//double x_diff = p2.x - p1.x;
	//double y_diff = p2.y - p2.y;
	//
	//return
	//	std::sqrt(x_diff*x_diff + y_diff*y_diff);
	return cv::norm(p1 - p2);
}

void skip_n_frames(cv::VideoCapture& capturer, int frames_to_skip) {
	if (0) {
		while (frames_to_skip-- > 0) {
			cv::Mat buf;
			{ capturer >> buf; if (buf.empty()) { break; } }
		}
	}

	if (1) {
		capturer.set(cv::CAP_PROP_POS_FRAMES, frames_to_skip);
	}
}

template<typename Container = std::deque<Point>>
void max_points_in_circle(Container points, double radius, double& containing_percentage) {
	radius *= radius;
	for (auto& point : points) {

	}
}

int main() {

	cv::VideoCapture cap("hand2.mp4");

	// Check if camera opened successfully
	if (!cap.isOpened()) {
		std::cout << "Error opening video stream or file" << std::endl;
		return -1;
	}

	enum KEY_BINDING { ESC = 27 };

	cv::Mat prevImage;
	{ cap >> prevImage; if (prevImage.empty()) { return 1; } }
	cv::cvtColor(prevImage, prevImage, cv::COLOR_BGR2GRAY);

	int width = prevImage.cols, height = prevImage.rows;

	IVec ROI, ROI_dynamic;
	PointVec points_default, prev_points, result_points;
	auto x_ratio = 0.5, y_ratio = 0.1;
	auto x_point_num = 50, y_point_num = 50;
	result_points.reserve(x_point_num*y_point_num);

	get_ROI_by_ratio(width, height, x_ratio, y_ratio, ROI);
	get_points_in_ROI(x_point_num, y_point_num, ROI, points_default);

	prev_points = points_default;

	auto winSize = cv::Size{ 21,21 };
	cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);

	//movement less than this pixel value will be ignored
	auto mvt_thres_min = 0.0;
	auto mvt_thres_max = 10.0;
	//mvt_thres_min *= mvt_thres_min;
	//mvt_thres_max *= mvt_thres_max;

	int frames_to_skip = 1000;
	skip_n_frames(cap, frames_to_skip);

	//reset_ROI()

	enum FIND_MODE { EVERY_PTS = 0, OVER_THRES, MAXIMUM_THRES };


	//FIND_MODE find_mode = EVERY_PTS;
	//FIND_MODE find_mode = OVER_THRES;
	FIND_MODE find_mode = MAXIMUM_THRES;

	int max_point_num = 10;
	std::deque<Point> maxPoints(max_point_num);
	std::deque<double> maxDists(max_point_num, -1.0);
	IVec def_pt_idx;
	if (find_mode == EVERY_PTS) {
		def_pt_idx.resize(x_point_num*y_point_num);
		std::iota(std::begin(def_pt_idx), std::end(def_pt_idx), 0);
	}

	while (1) {

		cv::Mat curImage, img_for_disp;
		{ cap >> img_for_disp; if (img_for_disp.empty()) { break; } }
		cv::cvtColor(img_for_disp, curImage, cv::COLOR_BGR2GRAY);
		//cv::GaussianBlur(curImage, curImage, cv::Size{ 3,3 }, 0.5);

		PointVec resultPoints;
		std::vector<uchar> status;
		std::vector<float> err;
		cv::calcOpticalFlowPyrLK(prevImage, curImage, points_default, resultPoints, status, err, winSize, 3, termcrit, 0, 0.001);

		Point point_to_draw(-1.0, -1.0);
		result_points.clear();
		maxPoints.clear();
		maxPoints.resize(max_point_num);
		maxDists = std::deque<double>(max_point_num, -1.0);

		//int pt_idx = 0;
		//for (auto& point : resultPoints) {
		for (int pt_idx = 0; pt_idx < resultPoints.size(); pt_idx++) {

			if (!status[pt_idx]) { continue; }


			const auto& point = resultPoints[pt_idx];
			const auto& point_def = points_default[pt_idx];

			if (point.x < 0 || point.y < 0 || point.x >= width || point.y >= height) { continue; }
			auto cur_dist = dist_2pts(point_def, point);

			switch (find_mode) {
			case EVERY_PTS:
				//img_for_disp.at<cv::Vec3b>(point) = cv::Vec3b(0,0,255);
				result_points.push_back(point);
				break;
			case OVER_THRES:
				if (cur_dist >= mvt_thres_min &&
					cur_dist <= mvt_thres_max) {
					//circle(img_for_disp, point, 3, cv::Scalar(0, 0, 255), -1, 8);
					result_points.push_back(point);
					def_pt_idx.push_back(pt_idx);
				}
				break;
			case MAXIMUM_THRES:
				if (cur_dist >= mvt_thres_min &&
					cur_dist <= mvt_thres_max &&
					cur_dist > maxDists.back()) {

					cout << "cur_dist : [" << cur_dist << "]" << endl;;

					maxDists.push_back(cur_dist);
					maxPoints.push_back(point);
					maxDists.pop_front();
					maxPoints.pop_front();
					def_pt_idx.push_back(pt_idx);
				}
				break;
			}
		}
		cout << "==============================================================================" << endl;
		if (find_mode == MAXIMUM_THRES) {
			result_points = PointVec(maxPoints.begin(), maxPoints.end());
		}

		//draw line from default points to dest
		{
			//int pt_idx = 0;
			//for (auto& point : result_points) {
			for (int pt_idx = 0; pt_idx < result_points.size(); pt_idx++) {
				if (pt_idx >= def_pt_idx.size()) { continue; }
				const auto& point = result_points[pt_idx];
				const auto& point_def = result_points[def_pt_idx[pt_idx]];
				cv::line(img_for_disp, point_def, point, cv::Vec3i(0, 255, 0));
			}
		}

		//draw maximum distance
		if (find_mode == MAXIMUM_THRES) {
			for (auto& point : result_points) {
				if (point.x >= 0 && point.y >= 0) {
					circle(img_for_disp, point, 5, cv::Scalar(0, 0, 255), -1, 8);
				}
			}
		}



		prevImage = curImage;

		// Display the resulting frame
		cv::imshow("Frame", img_for_disp);

		// Press  ESC on keyboard to exit
		char c = (char)cv::waitKey(25);
		if (c == ESC)
			break;
	}

	// When everything done, release the video capture object
	cap.release();

	// Closes all the frames
	cv::destroyAllWindows();

	std::cout << "program finished" << std::endl;

	return 0;
}
#endif // MINE




#ifdef DO_OPENCV
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <ctype.h>

using namespace cv;
using namespace std;

static void help()
{
	// print a welcome message, and the OpenCV version
	cout << "\nThis is a demo of Lukas-Kanade optical flow lkdemo(),\n"
		"Using OpenCV version " << CV_VERSION << endl;
	cout << "\nIt uses camera by default, but you can provide a path to video as an argument.\n";
	cout << "\nHot keys: \n"
		"\tESC - quit the program\n"
		"\tr - auto-initialize tracking\n"
		"\tc - delete all the points\n"
		"\tn - switch the \"night\" mode on/off\n"
		"To add/remove a feature point click it\n" << endl;
}

Point2f point;
bool addRemovePt = false;

static void onMouse(int event, int x, int y, int /*flags*/, void* /*param*/)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		point = Point2f((float)x, (float)y);
		addRemovePt = true;
	}
}

int main(int argc, char** argv)
{
	VideoCapture cap;
	TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
	Size subPixWinSize(10, 10), winSize(31, 31);

	const int MAX_COUNT = 500;
	bool needToInit = false;
	bool nightMode = false;

	help();
	cv::CommandLineParser parser(argc, argv, "{@input|0|}");
	string input = parser.get<string>("@input");

	if (input.size() == 1 && isdigit(input[0]))
		cap.open(input[0] - '0');
	else
		cap.open(input);

	if (!cap.isOpened())
	{
		cout << "Could not initialize capturing...\n";
		return 0;
	}

	namedWindow("LK Demo", 1);
	setMouseCallback("LK Demo", onMouse, 0);

	Mat gray, prevGray, image, frame;
	vector<Point2f> points[2];

	for (;;)
	{
		cap >> frame;
		if (frame.empty())
			break;

		frame.copyTo(image);
		cvtColor(image, gray, COLOR_BGR2GRAY);

		if (nightMode)
			image = Scalar::all(0);

		if (needToInit)
		{
			// automatic initialization
			goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 3, 0, 0.04);
			cornerSubPix(gray, points[1], subPixWinSize, Size(-1, -1), termcrit);
			addRemovePt = false;
		}
		else if (!points[0].empty())
		{
			vector<uchar> status;
			vector<float> err;
			if (prevGray.empty())
				gray.copyTo(prevGray);
			calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
				3, termcrit, 0, 0.001);
			size_t i, k;
			for (i = k = 0; i < points[1].size(); i++)
			{
				if (addRemovePt)
				{
					if (norm(point - points[1][i]) <= 5)
					{
						addRemovePt = false;
						continue;
					}
				}

				if (!status[i])
					continue;

				points[1][k++] = points[1][i];
				circle(image, points[1][i], 3, Scalar(0, 255, 0), -1, 8);
			}
			points[1].resize(k);
		}

		if (addRemovePt && points[1].size() < (size_t)MAX_COUNT)
		{
			vector<Point2f> tmp;
			tmp.push_back(point);
			cornerSubPix(gray, tmp, winSize, Size(-1, -1), termcrit);
			points[1].push_back(tmp[0]);
			addRemovePt = false;
		}

		needToInit = false;
		//goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 3, 0, 0.04);
		imshow("LK Demo", image);

		char c = (char)waitKey(10);
		if (c == 27)
			break;
		switch (c)
		{
		case 'r':
			needToInit = true;
			break;
		case 'c':
			points[0].clear();
			points[1].clear();
			break;
		case 'n':
			nightMode = !nightMode;
			break;
		}

		std::swap(points[1], points[0]);
		cv::swap(prevGray, gray);
	}

	return 0;
}
#endif