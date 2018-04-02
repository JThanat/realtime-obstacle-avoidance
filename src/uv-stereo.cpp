/*
* Stereo References: https://github.com/opencv/opencv/blob/master/samples/cpp/stereo_match.cpp
*/
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/utility.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

using namespace cv;
using namespace std;

Mat calculate_udisparity(Mat disp_img, int max_disp, Size image_size) 
{
    Mat uhist_vis = Mat::zeros(max_disp, image_size.width, CV_8UC1);
    Mat ublack_mask = Mat::zeros(max_disp, image_size.width, CV_8UC1);
    Mat hist, tmp_col;
    int i,j,k;
    int channels[] = {0};
    int hist_size[] = {max_disp};
    float tmp_range[] = {0.0, (float) max_disp};
    const float* ranges[] = {tmp_range};
    

    for (i=0; i < image_size.width ; i++)
    {
        /*
        void calcHist( const Mat* images, int nimages,
                          const int* channels, InputArray mask,
                          OutputArray hist, int dims, const int* histSize,
                          const float** ranges, bool uniform = true, bool accumulate = false );
        */
        tmp_col = disp_img.col(i);
        calcHist( &tmp_col, 1, channels, Mat(), hist, 1, hist_size, ranges, true, false);
        hist.col(0).copyTo(uhist_vis.col(i));
    }
    uhist_vis = uhist_vis > max_disp * 0.1;
    return uhist_vis;
}

int main(void)
{

    enum
    {
        STEREO_BM = 0,
        STEREO_SGBM = 1,
        STEREO_HH = 2,
        STEREO_VAR = 3,
        STEREO_3WAY = 4
    };

    std::string img_left_path, img_right_path;
    Mat left_img, right_img, cropped_left, cropped_right;
    int h, w;
    int i, j, k;
    int VROIX, VROIY, VROIW, VROIH;
    double sf;
    int alg;
    int64 t;

    int SADWindowSize, numberOfDisparities;
    Size image_size;

    Mat disp, disp8;
    Mat R, T, R1, R2, P1, P2, Q;
    Mat camera_matrix[2], dist_coeffs[2];
    Mat rimg[2], cimg;
    Mat rmap[2][2];
    Rect validRoi[2];

    vector<Point2f> imagePoints[2];
    vector<Point3f> objectPoints;

    Mat uhist_vis;

    alg = STEREO_SGBM;
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 16, 3);

    FileStorage fs("./extrinsics.yml", FileStorage::READ);

    // img path
    img_left_path = "../calib_img/debayer-img.jpg";
    img_right_path = "../calib_img/debayer-img2.jpg";

    left_img = imread(img_left_path, CV_LOAD_IMAGE_GRAYSCALE);
    right_img = imread(img_right_path, CV_LOAD_IMAGE_GRAYSCALE);

    image_size = left_img.size();
    // Setup Extrinsics Camera Matrix
    if (fs.isOpened())
    {
        fs["R"] >> R;
        fs["T"] >> T;
        fs["R1"] >> R1;
        fs["R2"] >> R2;
        fs["P1"] >> P1;
        fs["P2"] >> P2;
        fs["Q"] >> Q;
        fs["VROIX"] >> VROIX;
        fs["VROIY"] >> VROIY;
        fs["VROIW"] >> VROIW;
        fs["VROIH"] >> VROIH;

        // cout << R << T << R1 << R2 << P1 << P2 << Q << endl;
        fs.release();
    }
    else
        cout << "Error: can not read the extrinsic parameters\n";

    // Setup Intrinsics Camera Matrix
    fs.open("./intrinsics.yml", FileStorage::READ);
    if (fs.isOpened())
    {
        fs["M1"] >> camera_matrix[0];
        fs["M2"] >> camera_matrix[1];
        fs["D1"] >> dist_coeffs[0];
        fs["D2"] >> dist_coeffs[1];
        fs.release();
    }
    // set up other values
    sf = 600. / MAX(image_size.width, image_size.height);
    w = cvRound(image_size.width * sf);
    h = cvRound(image_size.height * sf);

    cout << w << " " << h << endl;

    // undistort and rectify
    t = getTickCount();
    initUndistortRectifyMap(camera_matrix[0], dist_coeffs[0], R1, P1, image_size, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(camera_matrix[1], dist_coeffs[1], R2, P2, image_size, CV_16SC2, rmap[1][0], rmap[1][1]);

    remap(left_img, rimg[0], rmap[0][0], rmap[0][1], INTER_LINEAR);
    remap(right_img, rimg[1], rmap[1][0], rmap[1][1], INTER_LINEAR);

    // use second region of interest because it is smaller for this specific camera calibration
    Rect vroi(cvRound(VROIX * sf), cvRound(VROIY * sf),
              cvRound(VROIW * sf), cvRound(VROIH * sf));
    
    resize(rimg[0], rimg[0], Size(w,h), 0,0, INTER_AREA);
    resize(rimg[1], rimg[1], Size(w,h), 0,0, INTER_AREA);
    cropped_left = rimg[0](vroi);
    cropped_right = rimg[1](vroi);

    t = getTickCount() - t;
    printf("Rectification Time elapsed: %fms\n", t * 1000 / getTickFrequency());

    // find disparity
    // SADWindowSize = 11;
    // numberOfDisparities = 16;

    sgbm->setPreFilterCap(63);
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->setBlockSize(sgbmWinSize);
    int cn = cropped_left.channels();

    image_size = cropped_left.size();
    numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((image_size.width / 8) + 15) & -16;
    printf("num of disp %d\n", numberOfDisparities);
    sgbm->setP1(8 * cn * sgbmWinSize * sgbmWinSize);
    sgbm->setP2(32 * cn * sgbmWinSize * sgbmWinSize);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);

    sgbm->setMode(StereoSGBM::MODE_SGBM);

    t = getTickCount();
    sgbm->compute(cropped_left, cropped_right, disp);
    t = getTickCount() - t;
    printf("Disparity Time elapsed: %fms\n", t * 1000 / getTickFrequency());

    if (alg != STEREO_VAR)
        disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities * 16.));
    else
        disp.convertTo(disp8, CV_8U);

    uhist_vis = calculate_udisparity(disp8, numberOfDisparities + 0, image_size);
    
    // drawing canvas
    Mat canvas;
    w = cropped_left.size().width;
    h = cropped_left.size().height;
    canvas.create(h, w*2, CV_8UC1);
    cropped_left.copyTo(canvas.rowRange(0,h).colRange(0,w));
    cropped_right.copyTo(canvas.rowRange(0,h).colRange(w,w*2));
    cvtColor(canvas, canvas, CV_GRAY2BGR);
    for( j = 0; j < canvas.rows; j += 16 )
        line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);

    imshow("canvas", canvas);
    imshow("disp", disp8);
    imshow("uhist", uhist_vis);
    waitKey(0);

    // uv map


    // connect u map

    // draw elipse

    // project back

    return 0;
}