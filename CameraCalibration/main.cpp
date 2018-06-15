#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <cstdlib>
#include <cstdio>
#include <zconf.h>

using namespace cv;
using namespace std;

int main() {
    int numBoards = 0;
    int numCornersHor;
    int numCornersVer;

    cout << ("Enter number of corners along width: ");
    cin >> numCornersHor;

    cout << ("Enter number of corners along height: ");
    cin >> numCornersVer;

    cout << ("Enter number of boards: ");
    cin >> numBoards;

    int numSquares = numCornersHor * numCornersVer;
    Size board_sz = Size(numCornersHor, numCornersVer);
    VideoCapture capture(0);

    vector<vector<Point3f>> object_points;
    vector<vector<Point2f>> image_points;

    vector<Point2f> corners;
    int successes = 0;

    Mat image;
    Mat gray_image;
    capture >> image;

    vector<Point3f> obj;
    for (int j = 0; j < numSquares; j++)
        obj.emplace_back(j / numCornersHor, j % numCornersHor, 0.0f);

    while (successes < numBoards) {

        usleep(1000000);
        cvtColor(image, gray_image, COLOR_BGR2GRAY);

        bool found = findChessboardCorners(image, board_sz, corners,
                                           CALIB_CB_ADAPTIVE_THRESH +
                                           CALIB_CB_FILTER_QUADS);

        if (found) {
            cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1),
                         TermCriteria(TermCriteria::EPS +
                                      TermCriteria::MAX_ITER, 30, 0.1));
            drawChessboardCorners(gray_image, board_sz, corners, found);
            image_points.push_back(corners);
            object_points.push_back(obj);
            cout << "Snap stored with number " << successes << std::endl;

            successes++;

            if (successes >= numBoards)
                break;
        }

        imshow("Colored", image);
        imshow("Grayscale", gray_image);

        capture >> image;

        int key = waitKey(1);

        if (key == 27)
            return 0;

    }
    Mat intrinsic = Mat(3, 3, CV_32F);
    Mat distCoeffs;
    vector<Mat> rvecs;
    vector<Mat> tvecs;

    intrinsic.ptr<float>(0)[0] = 1;
    intrinsic.ptr<float>(1)[1] = 1;

    calibrateCamera(object_points, image_points, image.size(), intrinsic,
                    distCoeffs, rvecs, tvecs);

    Mat imageUndistorted;
    std::cout << intrinsic << "\n" << std::endl;
    while (true) {
        capture >> image;
        undistort(image, imageUndistorted, intrinsic, distCoeffs);

        imshow("Before", image);
        imshow("After", imageUndistorted);

        int key = waitKey(1);
        if (key == 27)
            break;
    }

    capture.release();

    return 0;
}