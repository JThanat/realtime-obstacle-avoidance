/*
* Stereo References: https://github.com/opencv/opencv/blob/master/samples/cpp/stereo_match.cpp

Note: for Matrix and Point Coordinate System
0/0---column(j)--->   0/0---X--->
|                     |
row(i)    Mat         Y   Point
|                     |
v                     v
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
#include <stack> 
#include <queue>
#include <utility>

#include "umap_util.hpp"
#include "wp_planning.hpp"

using namespace cv;
using namespace std;
using namespace umap_utility;
using namespace wp;

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
    Mat left_img_debayer(1842/2, 2432/2, CV_8UC1);
    Mat right_img_debayer(1842/2, 2432/2, CV_8UC1);
    int h, w;
    int i, j, k;
    int VROIX, VROIY, VROIW, VROIH;
    double sf;
    int alg;
    int obj_count;
    int64 t;

    double *pe1, *pe2, *se1, *se2;
    double f,b, GYb;
    int current_x, current_y;

    int SADWindowSize, numberOfDisparities;
    Size image_size;

    Mat disp(600,800, CV_8UC3);
    Mat disp8(600,800, CV_8UC3);
    Mat R, T, R1, R2, P1, P2, Q;
    Mat camera_matrix[2], dist_coeffs[2];
    Mat rimg[2], cimg;
    Mat rmap[2][2];
    Rect validRoi[2];

    vector<Point2f> imagePoints[2];
    vector<Point3f> objectPoints;

    vector<ellipse_desc> ellipse_list;
    vector< pair<double, double> > waypoints;
    vector< pair<double, double> > waypoints_pub(20);

    Mat uhist_vis;

    alg = STEREO_SGBM;
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 16, 3);

    FileStorage fs("./extrinsics.yml", FileStorage::READ);

    // img path
    char *imagedata = NULL;
    int framesize = 2432*1842;
    imagedata = (char*) malloc (sizeof(char) * framesize * 2);

    img_left_path = "../data/raw_image/left1.raw";
    img_right_path = "../data/raw_image/right1.raw";
    FILE *fp = NULL;
    
    fp = fopen(img_left_path.c_str(), "rb");
    fread(imagedata, sizeof(char), framesize*2, fp);
    
    left_img.create(1842, 2432, CV_16UC1);
    memcpy(left_img.data, imagedata, framesize*2);
    fclose(fp);

    fp = fopen(img_right_path.c_str(), "rb");
    fread(imagedata, sizeof(char), framesize*2, fp);
    
    right_img.create(1842, 2432, CV_16UC1);
    memcpy(right_img.data, imagedata, framesize*2);
    
    free(imagedata);
    fclose(fp);


    t = getTickCount();
    // Copy the data into an OpenCV Mat structure
    uint8_t *ptr_i, *ptr_i1, *dbptr;
    uint8_t *rptr_i, *rptr_i1, *rdbptr;
    int red,green,blue;
    int ib, jb;

    imwrite("./bayer_left_img.jpg", left_img);
    // left_img.convertTo(left_img, CV_8UC1, 1);
    // right_img.convertTo(right_img, CV_8UC1, 1/4.0);

    for (i = 0 ; i < left_img.rows/2 ; i++)
    {
        ib = i*2;
        ptr_i = left_img.ptr<uint8_t>(ib);
        ptr_i1 = left_img.ptr<uint8_t>(ib+1);
        dbptr = left_img_debayer.ptr<uint8_t>(i);

        rptr_i = right_img.ptr<uint8_t>(ib);
        rptr_i1 = right_img.ptr<uint8_t>(ib+1);
        rdbptr = right_img_debayer.ptr<uint8_t>(i);

        for ( j = 0 ; j < left_img.cols/2 ; j++)
        {
            jb = j*2;
            // ptr_i[jb] = ptr_i[jb] >> 2;
            // ptr_i[jb+1] = ptr_i[jb+1] >> 2;
            // ptr_i1[jb] = ptr_i1[jb] >> 2;
            // ptr_i1[jb+1] = ptr_i1[jb+1] >> 2;

            // rptr_i[jb] = rptr_i[jb] >> 2;
            // rptr_i[jb+1] = rptr_i[jb+1] >> 2;
            // rptr_i1[jb] = rptr_i1[jb] >> 2;
            // rptr_i1[jb+1] = rptr_i1[jb+1] >> 2;

            green = int((ptr_i[jb] + ptr_i1[jb+1])/2);
            red = int(ptr_i[jb+1]);
            blue = int(ptr_i1[jb]);
            dbptr[j] = uint8_t(sqrt(0.299*red*red + 0.587*green*green + 0.114*blue*blue));

            green = int(rptr_i[jb] + rptr_i1[jb+1])/2;
            red = int(rptr_i[jb+1]);
            blue = int(rptr_i1[jb]);
            rdbptr[j] = uint8_t(sqrt(0.299*red*red + 0.587*green*green + 0.114*blue*blue));
        }
    }
    t = getTickCount() - t;
    printf("loop time: %fms\n", t * 1000 / getTickFrequency());

    // imwrite("./left_img_debayer.jpg", left_img_debayer);
    // imwrite("./right_img_debayer.jpg", right_img_debayer);
    // waitKey(0);

    // // Decode the Bayer data to RGB but keep using 16 bits per channel
    // cv::Mat mat16uc3_rgb(left_img.rows, left_img.cols, CV_16UC3);
    // cv::cvtColor(left_img, mat16uc3_rgb, cv::COLOR_BayerGR2RGB);

    // // Convert the 16-bit per channel RGB image to 8-bit per channel
    // cv::Mat mat8uc3_rgb(left_img.rows, left_img.cols, CV_8UC3);
    // mat16uc3_rgb.convertTo(mat8uc3_rgb, CV_8UC3, 1.0/256);
    
    // imshow("mat8uc3_rgb", mat8uc3_rgb);
    // waitKey(0);

    return 0;

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

    // undistort and rectify
    t = getTickCount();
    initUndistortRectifyMap(camera_matrix[0], dist_coeffs[0], R1, P1, image_size, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(camera_matrix[1], dist_coeffs[1], R2, P2, image_size, CV_16SC2, rmap[1][0], rmap[1][1]);

    remap(left_img, rimg[0], rmap[0][0], rmap[0][1], INTER_LINEAR);
    remap(right_img, rimg[1], rmap[1][0], rmap[1][1], INTER_LINEAR);

    // use second region of interest because it is smaller for this specific camera calibration
    Rect vroi(cvRound(VROIX * sf), cvRound(VROIY * sf),
              cvRound(VROIW * sf), cvRound(VROIH * sf));

    resize(rimg[0], rimg[0], Size(w, h), 0, 0, INTER_AREA);
    resize(rimg[1], rimg[1], Size(w, h), 0, 0, INTER_AREA);
    cropped_left = rimg[0](vroi);
    cropped_right = rimg[1](vroi);

    t = getTickCount() - t;
    printf("Rectification Time elapsed: %fms\n", t * 1000 / getTickFrequency());

    // find disparity
    SADWindowSize = 3;
    numberOfDisparities = 0;

    sgbm->setPreFilterCap(63);
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->setBlockSize(sgbmWinSize);
    int cn = cropped_left.channels();

    image_size = cropped_left.size();
    numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((image_size.width / 8) + 15) & -16;
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


    double min, max;
    minMaxLoc(disp8, &min, &max, NULL, NULL);
    cout << min << " " << max << " " << numberOfDisparities << endl;
    
    // u-map and line connection
    ellipse_list = calculate_udisparity(disp8, max, image_size, obj_count);

    // drawing canvas
    // Mat canvas;
    // w = cropped_left.size().width;
    // h = cropped_left.size().height;
    // canvas.create(h, w * 2, CV_8UC1);
    // cropped_left.copyTo(canvas.rowRange(0, h).colRange(0, w));
    // cropped_right.copyTo(canvas.rowRange(0, h).colRange(w, w * 2));
    // cvtColor(canvas, canvas, CV_GRAY2BGR);
    // for (j = 0; j < canvas.rows; j += 16)
    //     line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);

    // imshow("canvas", canvas);
    // imshow("disp", disp8);
    // // imshow("uhist", uhist_vis);
    // waitKey(0);

    /*
    Drawing Ellipse Section
    */

    f = 30.23622; // in pixel 1 millimeter = 3.779528 pixel
    b = 15;   // in cm
      
    // TODO: update current position
    current_x = 3000; // need update
    current_y = 0; // need update
    Mat obstacle_map(2000, 6000, CV_8UC3);

    GYb = 0; // 0 degree for now
    Mat GRb = (Mat_<double>(2, 2) << cos(GYb * M_PI / 180), -sin(GYb * M_PI / 180), sin(GYb * M_PI / 180), cos(GYb * M_PI / 180));
    Mat GPb = (Mat_<double>(2, 1) << current_x, current_y);
    
    for (i = 0; i < obj_count; ++i)
    {
        // cout << "ellipse "<< i << endl; 
        ellipse_list[i].u1 = ellipse_list[i].u1 - image_size.width/2; // set position of the drone at the center of the image
        ellipse_list[i].u2 = ellipse_list[i].u2 - image_size.width/2;
        ellipse_list[i].d1 = ellipse_list[i].d1/16;
        ellipse_list[i].d2 = ellipse_list[i].d2/16;

        // cout << "after scaling: " << ellipse_list[i].u1 << " " << ellipse_list[i].u2 << " " << ellipse_list[i].d1 << " " << ellipse_list[i].d2 << endl;

        ellipse_list[i].BPe = (Mat_<double>(2, 1) << b*(ellipse_list[i].u1 + ellipse_list[i].u2)/(2*ellipse_list[i].d2), f*b/(ellipse_list[i].d2));
        ellipse_list[i].BSe = (Mat_<double>(2, 1) << b*(ellipse_list[i].u2 - ellipse_list[i].u1)/(2*ellipse_list[i].d2), f*b/(ellipse_list[i].d2) - f*b/(ellipse_list[i].d1));
        ellipse_list[i].GYe = GYb;
        ellipse_list[i].ESe = (Mat_<double>(2, 1) << b*(ellipse_list[i].u2 - ellipse_list[i].u1)/(2*ellipse_list[i].d2), f*b/(ellipse_list[i].d2) - f*b/(ellipse_list[i].d1));
        ellipse_list[i].GPe = GRb * ellipse_list[i].BPe + GPb;
        ellipse_list[i].GSe = GRb * ellipse_list[i].BSe + GPb;

        // drawing ellipse
        pe1 = ellipse_list[i].GPe.ptr<double>(0);
        pe2 = ellipse_list[i].GPe.ptr<double>(1);

        se1 = ellipse_list[i].BSe.ptr<double>(0);
        se2 = ellipse_list[i].BSe.ptr<double>(1);
        se1[0] = se1[0] + 50; // 50 cm boundary
        se2[0] = se2[0] + 50; // 50 cm boundary
        ellipse(obstacle_map, Point(cvRound(pe1[0]),cvRound(2000 - pe2[0])), Size(cvRound(se1[0] - 50),cvRound(se2[0] - 50)), 0, 0, 360, Scalar(0,0,255),2);
        ellipse(obstacle_map, Point(cvRound(pe1[0]),cvRound(2000 - pe2[0])), Size(cvRound(se1[0]),cvRound(se2[0])), 0, 0, 360, Scalar(0,255,0),2);
        cout << "drawn: " << pe1[0] << " " << pe2[0] << " " << 2*se1[0] << " " << 2*se2[0] << endl;
    }
    // line(obstacle_map, Point(3000,0), Point(3000,2000), Scalar(0,0,255));
    /*
    Generate Waypoint 
    */
    for (j = 1 ; j <= 11 ; j++)
    {
        waypoints.clear();
        for (i = 0 ; i < 200 ; i++)
        {
            waypoints.push_back(make_pair(500*j,10*i));
        }
        t = getTickCount();
        waypoints_pub = waypoint_checking(waypoints, ellipse_list, obj_count);
        t = getTickCount() - t;
        printf("generate waypoint time: %fms\n", t * 1000 / getTickFrequency());    

        for ( i = 0 ; i < waypoints_pub.size() - 1 ; i++)
        {
            // cout << waypoints_pub[i].first << " " << waypoints_pub[i].second << endl;
            line(obstacle_map, Point(cvRound(waypoints_pub[i].first), cvRound(2000 - waypoints_pub[i].second)), Point(cvRound(waypoints_pub[i+1].first), cvRound(2000 - waypoints_pub[i+1].second)), Scalar(0,0,255), 2);
        }
    }
    imwrite("./obstacle_map.jpg", obstacle_map);

    return 0;
}