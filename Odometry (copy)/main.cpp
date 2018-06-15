#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <vector>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <boost/algorithm/string.hpp>
#include <string>
#include <fcntl.h>

using namespace std;
using namespace cv;


#define MIN_NUM_FEAT 2000


#define earthRadiusKm 6371.0
#define BUFLEN 512
using namespace std;

double deg2rad(double deg) {
    return (deg * M_PI / 180);
}

double distanceEarth(double lat1d, double lon1d, double lat2d, double lon2d) {
    double lat1r, lon1r, lat2r, lon2r, u, v;
    lat1r = deg2rad(lat1d);
    lon1r = deg2rad(lon1d);
    lat2r = deg2rad(lat2d);
    lon2r = deg2rad(lon2d);
    u = sin((lat2r - lat1r) / 2);
    v = sin((lon2r - lon1r) / 2);
    return 2.0 * earthRadiusKm *
           asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v));
}


vector<string> split(const string &str, const string &delim) {
    vector<string> tokens;
    size_t prev = 0, pos = 0;
    do {
        pos = str.find(delim, prev);
        if (pos == string::npos) pos = str.length();
        string token = str.substr(prev, pos - prev);
        if (!token.empty()) tokens.push_back(token);
        prev = pos + delim.length();
    } while (pos < str.length() && prev < str.length());
    return tokens;
}

/**
 * img1 : prevImg
 * img2 : nextImg
 * points1 : prevPts
 * points2 : nextPts
 **/

Point2f operator*(cv::Mat M, const cv::Point2f &p) {
    cv::Mat_<double> src(3/*rows*/, 1 /* cols */);

    src(0, 0) = p.x;
    src(1, 0) = p.y;
    src(2, 0) = 1.0;


    if (M.type() == 0) {
        std::cout << "#########\n";
        std::cout << M.type() << " " << src.type() << std::endl;
        return p;
    }
    cv::Mat_<double> dst = M * src; //USE MATRIX ALGEBRA
    return cv::Point2f(dst(0, 0), dst(1, 0));
}

void featureTracking(Mat img1, Mat img2, vector<Point2f> &points1,
                     vector<Point2f> &points2, vector<uchar> &status) {
    vector<float> err;
    Size winSize = Size(21, 21); // Dimensiunea unde se cauta colturile
    TermCriteria termCriteria = TermCriteria(TermCriteria::COUNT +
                                             TermCriteria::EPS, 30, 0.01);
    calcOpticalFlowPyrLK(img1, img2, points1, points2, status, err, winSize,
                         3, termCriteria, 0, 0.001);

    int indexCorrection = 0;
    for (int i = 0; i < status.size(); ++i) {
        Point2f point = points2.at(i - indexCorrection);
        if ((status.at(i) == 0) || point.x < 0 || point.y < 0) {
            if (point.x < 0 || point.y < 0)
                status.at(i) = 0;
            points1.erase(points1.begin() + (i - indexCorrection));
            points2.erase(points2.begin() + (i - indexCorrection));
            indexCorrection++;
        }
    }
}

void featureDetection(Mat img1, vector<Point2f> &points1) {
    vector<KeyPoint> keypoints;
    int fastThreshold = 20;
    bool nonmaxSupression = true;
    FAST(img1, keypoints, fastThreshold, nonmaxSupression);
    KeyPoint::convert(keypoints, points1, vector<int>());
}

int main(int argc, char *argv[]) {
    //UDP Server
    int PORT;
    if (argc < 2) {
        std::cout << "./Odometry <port>\n";
        return -1;
    }
    PORT = atoi(argv[1]);
    std::cout << PORT << std::endl;

    // FD Sets
    fd_set readFds;
    fd_set tempFds;
    FD_ZERO(&readFds);
    FD_ZERO(&tempFds);

    // UDP socket
    struct sockaddr_in serverAddr, otherAddr;
    int s;
    ssize_t recvLen;
    socklen_t slen = sizeof(otherAddr);

    char buf[BUFLEN];
    if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
        std::cout << "Error creating socket\n";
        return -1;
    }

    fcntl(s, F_SETFL, O_NONBLOCK);

    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(PORT);
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(s, (sockaddr *) &serverAddr, sizeof(serverAddr)) == -1) {
        std::cout << "Bind error\n";
        return -1;
    }

    listen(s, 1);
    FD_SET(s, &readFds);

    int fdmax = s;

    std::string delimiter = ",";
    size_t start = 0;
    size_t end = 0;
    std::string token;


    // Image matrices
    Mat image1Colored, image2Colored;
    Mat image1Gray, image2Gray;

    VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cout << "Error opening file\n";
        return -1;
    }

    cap >> image1Colored >> image2Colored; //Initialize with first 2 frames

    // Convert images to grayscale
    cvtColor(image1Colored, image1Gray, COLOR_BGR2GRAY);
    cvtColor(image2Colored, image2Gray, COLOR_BGR2GRAY);

    // Used for feature detection and tracking
    vector<Point2f> points1, points2;
    featureDetection(image1Gray, points1);
    vector<uchar> status;
    featureTracking(image1Gray, image2Gray, points1, points2, status);

    // Origin point
    Point2f origin(0.0, 0.0);

    // Used in the iteration
    Mat prevImage = image2Gray;
    Mat currImage;
    vector<Point2f> prevFeatures = points2;
    vector<Point2f> currFeatures;

    // Windows used to display the camera image and the moving trajectory
    namedWindow("Camera", WINDOW_AUTOSIZE);
    namedWindow("Trajectory", WINDOW_AUTOSIZE);
    Mat trajectory = Mat::zeros(800, 800, CV_8UC3);

    // Transform matrix to apply to origin point
    Mat transformMatrix;

    // Timeout struct
    timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 1000;

    // Reference GPS point and current GPS point
    double refLat, refLon;
    double currentLat, currentLon;
    double walkedDistance;
    refLat = refLon = currentLat = currentLon = walkedDistance = 0.0;


    while (1) {
        while (1) {
            // Getting UDP data
            tempFds = readFds;
            if (select(fdmax + 1, &tempFds, NULL, NULL, &timeout) == -1) {
                std::cout << "Select error\n";
                fflush(stdout);
                return -1;
            }
            if (FD_ISSET(s, &tempFds)) {
                // Get data from UDP socket
                memset(buf, 0, sizeof(buf));

                recvLen = recvfrom(s, buf, BUFLEN, 0, (sockaddr *)
                                           &otherAddr,
                                   &slen);
                std::string bufString(buf);
                vector<string> fields = split(bufString, delimiter);
                if (fields[0] == "O") {
                    std::cout << "Orientation\n";
                    double x = std::stod(fields[3]);
                    double y = std::stod(fields[4]);
                    double z = std::stod(fields[5]);
                    std::cout << x << " " << y << " " << z << std::endl;
                } else if (fields[0] == "G") {
                    std::cout << "GPS\n";
                    currentLat = std::stod(fields[4]);
                    currentLon = std::stod(fields[5]);
                    std::cout << currentLat << " " << currentLon << std::endl;
                }
            } else
                break;
        }

        // Calculate walked distance in reference to the referenced lat/lon
        walkedDistance = distanceEarth(refLat, refLon, currentLat, currentLon);

        // Image processing
        Mat currImageColored;
        cap >> currImageColored;
        cvtColor(currImageColored, currImage, COLOR_BGR2GRAY);
        vector<uchar> status;
        featureTracking(prevImage, currImage, prevFeatures, currFeatures,
                        status);

        transformMatrix = estimateRigidTransform(prevFeatures, currFeatures,
                                                 false);
        origin = transformMatrix * origin;
        std::cout << "####\n";
        std::cout << transformMatrix << std::endl;

        //Kalman filter
        

        Mat prevPts(2, prevFeatures.size(), CV_64F);
        Mat currPts(2, currFeatures.size(), CV_64F);
        for (size_t i = 0; i < prevFeatures.size(); ++i) {
            prevPts.at<double>(0, i) = prevFeatures.at(i).x;
            prevPts.at<double>(1, i) = prevFeatures.at(i).y;
            currPts.at<double>(0, i) = currFeatures.at(i).x;
            currPts.at<double>(1, i) = currFeatures.at(i).y;
        }

        if (prevFeatures.size() < MIN_NUM_FEAT) {
            featureDetection(prevImage, prevFeatures);
            featureTracking(prevImage, currImage, prevFeatures, currFeatures,
                            status);
        }

        prevImage = currImage.clone();
        prevFeatures = currFeatures;


//        for (int i = 0; i < currFeatures.size(); ++i) {
//            circle(currImageColored, currFeatures.at(i), 1, CV_RGB(255, 0, 0),
//                   2);
//        }

        circle(trajectory, Point(origin.x / 1 + 300, origin.y / 1 + 100), 1,
               CV_RGB
               (255, 0, 0), 2);
        rectangle(trajectory, Point(10, 30), Point(550, 50), CV_RGB(0, 0, 0));

//        std::cout << origin.x << ' ' << origin.y << std::endl;

        imshow("Camera", currImageColored);
        imshow("Trajectory", trajectory);
        if (waitKey(1) == 27)
            break;
    }
    cap.release();

    return 0;
}