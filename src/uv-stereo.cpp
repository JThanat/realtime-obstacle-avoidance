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

using namespace cv;
using namespace std;

bool is_out_of_boud(int row, int col, int nrows, int ncols)
{
    return row >= 0 && row <= nrows && col >=0 && col <= ncols;
}

void add_neighbor(int i, int j, int dr, int dc, int maxr, int maxcol, Mat tmp_uline, Mat connected_component, queue< pair<int,int> >& neighbors, int mark)
{
    int m,n;
    uint8_t *p;
    int *cp;

    for (m = i + dr ; m >= i - dr ; --m)
    {
        if(m < 0 || m >= maxr) 
            continue;
        p = tmp_uline.ptr<uint8_t>(m);
        cp = connected_component.ptr<int>(m);
        for (n = j + dc ; n >= j - dc; --n)
        {
            if(n < 0 || n >= maxcol) continue;
            if (p[n] == 255 && cp[n] == 0)
            {
                // cout << "Adding " << m << "," << n << endl;
                neighbors.push(make_pair(m,n));
                cp[n] = -1;
            }
        }
    }
}

Mat calculate_udisparity(Mat disp_img, int max_disp, Size image_size)
{
    int64 t;
    uint8_t *p;
    int *cp, *connected_ptr;//finding neighbor pointer
    Mat uhist_vis = Mat::zeros(max_disp, image_size.width, CV_8UC1);
    Mat uline_mask = Mat::zeros(max_disp, image_size.width, CV_8UC1);
    Mat group_rect = Mat::zeros(max_disp, image_size.width, CV_8UC3);
    Mat connected_component = Mat::zeros(max_disp, image_size.width, CV_32SC1);
    Mat tmp_uline = Mat::zeros(max_disp, image_size.width, CV_8UC1);
    queue< pair<int,int> > neighbors;
    Mat hist, tmp_col;
    int i, j, k, start, end;
    int threshold = max_disp * 0.1;
    int accumulate_threshold;
    int channels[] = {0};
    int hist_size[] = {max_disp};
    float tmp_range[] = {0.0, (float)max_disp};
    const float *ranges[] = {tmp_range};

    // calculate map
    t = getTickCount();
    for (i = 0; i < image_size.width; i++)
    {
        /*
        void calcHist( const Mat* images, int nimages,
                          const int* channels, InputArray mask,
                          OutputArray hist, int dims, const int* histSize,
                          const float** ranges, bool uniform = true, bool accumulate = false );
        */
        tmp_col = disp_img.col(i);
        calcHist(&tmp_col, 1, channels, Mat(), hist, 1, hist_size, ranges, true, false);
        hist.col(0).copyTo(uhist_vis.col(i));
    }
    t = getTickCount() - t;
    // cout << "Calc UMap Time elapsed: " << t * 1000 / getTickFrequency() << "ms" << endl;
    t = getTickCount();

    // extract line
    for (i = 1; i < uhist_vis.rows; ++i) // we skip disparity 0 which is corrupted part
    {
        p = uhist_vis.ptr<uint8_t>(i);
        accumulate_threshold = 0;
        start = end = -1;
        for (j = 0; j < uhist_vis.cols; ++j)
        {
            if (p[j] >= threshold)
            {
                accumulate_threshold += p[j];
                if (start == -1) start = j;

            }
            else if (accumulate_threshold > 2 * max_disp)
            {
                end = j;
                // cout << "i:" << i << " " << start << " " << end << endl;
                line(uline_mask, Point(start, i), Point(end, i), Scalar(255, 0, 0), 1, 8);
                start = end = -1;
                accumulate_threshold = 0;
            }
            else 
            {
                start = end = -1;
                accumulate_threshold = 0;
            }
        }
    }
    t = getTickCount() - t;
    // cout << "Finding Line Time elapsed: " << t * 1000 / getTickFrequency() << "ms" << endl;

    // cvtColor(uline_mask, uline_mask, CV_GRAY2BGR);

    // connect line
    t = getTickCount();
    int mark = 1;
    int diff_inc = 1;
    int object_count = 0;
    int u,v;
    int maxd, maxc, mind, minc; // max-min disp and max-min col
    bool obj_found = false;
    pair<int,int> idx;
    uline_mask.copyTo(tmp_uline);
    for (i = tmp_uline.rows - 1; i > 0 ; --i) // scan from nearest
    {
        p = tmp_uline.ptr<uint8_t>(i);
        cp = connected_component.ptr<int>(i);
        for (j = 0; j < tmp_uline.cols; ++j)
        {
            if(p[j] == 255 && cp[j] == 0) // new white object
            {
                object_count += 1; // increase objects found
                neighbors.push(make_pair(i,j));
                cp[j] = -1;
                maxd = 0;
                mind = tmp_uline.rows + 1;
                maxc = 0; // x (cols)
                minc = tmp_uline.cols + 1;
                while(!neighbors.empty()) // find all connected component
                {
                    idx = neighbors.front();
                    connected_ptr = connected_component.ptr<int>(idx.first);
                    connected_ptr[idx.second] = mark;
                    neighbors.pop();
                    if (idx.first >= maxd) maxd = idx.first;
                    if (idx.first < mind) mind = idx.first;
                    if (idx.second >= maxc) maxc = idx.second;
                    if (idx.second < minc) minc = idx.second;
                    if (i >= max_disp*3/4)
                    {
                        add_neighbor(idx.first, idx.second, 10, 20, tmp_uline.rows, tmp_uline.cols, tmp_uline, connected_component, neighbors, mark);
                    } 
                    else if (i >= max_disp*2/4)
                    {
                        add_neighbor(idx.first, idx.second, 10, 20, tmp_uline.rows, tmp_uline.cols, tmp_uline, connected_component, neighbors, mark);
                    } 
                    else if (i >= max_disp*1/4)
                    {
                        add_neighbor(idx.first, idx.second, 5, 20, tmp_uline.rows, tmp_uline.cols, tmp_uline, connected_component, neighbors, mark);
                    }
                    else 
                    {
                        add_neighbor(idx.first, idx.second, 2, 4, tmp_uline.rows, tmp_uline.cols, tmp_uline, connected_component, neighbors, mark);
                    }
                }
                // cout << "object " << object_count << " " << minc << "," << maxc << " " << mind << "," << maxd << endl;
                rectangle(group_rect, Point(minc, maxd), Point(maxc, mind), Scalar(255,0,0));
                
                mark += diff_inc;
            }
        }
    }
    for (i = 0 ; i < 4 ; i++)
        line(group_rect, Point(0, i*max_disp/4), Point(group_rect.cols, i*max_disp/4), Scalar(0,0,255));
    cvtColor(uline_mask,uline_mask, CV_GRAY2BGR);
    for (i = 0 ; i < group_rect.rows ; i=i+10)
        line(uline_mask, Point(0, i), Point(group_rect.cols, i), Scalar(0,255,0));

    cout << "Number of object " << object_count << endl;

    t = getTickCount() - t;
    cout << "Connect the Line Time elapsed: " << t * 1000 / getTickFrequency() << "ms" << endl;
    imshow("uhist", uhist_vis);
    imshow("group_rect", group_rect);
    imshow("uline", uline_mask);
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
    // SADWindowSize = 11;
    // numberOfDisparities = 16;

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
    uhist_vis = calculate_udisparity(disp8, max, image_size);

    // drawing canvas
    Mat canvas;
    w = cropped_left.size().width;
    h = cropped_left.size().height;
    canvas.create(h, w * 2, CV_8UC1);
    cropped_left.copyTo(canvas.rowRange(0, h).colRange(0, w));
    cropped_right.copyTo(canvas.rowRange(0, h).colRange(w, w * 2));
    cvtColor(canvas, canvas, CV_GRAY2BGR);
    for (j = 0; j < canvas.rows; j += 16)
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