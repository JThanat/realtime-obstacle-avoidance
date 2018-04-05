#include "umap_util.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace umap_utility
{
    Mat calculate_udisparity(Mat disp_img, int max_disp, Size image_size)
    {
        int64 t;
        Mat uhist_vis = Mat::zeros(max_disp, image_size.width, CV_8UC1);
        Mat hist, tmp_col;
        int channels[] = {0};
        int hist_size[] = {max_disp};
        float tmp_range[] = {0.0, (float)max_disp};
        const float *ranges[] = {tmp_range};

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
        cout << "Calc UMap Time elapsed: " << t * 1000 / getTickFrequency() << "ms" << endl;
        cout << uhist_vis.depth() << endl;
        t = getTickCount();

        t = getTickCount() - t;
        cout << "Finding Line Time elapsed: " << t * 1000 / getTickFrequency() << "ms" << endl;
        imshow("uhist", uhist_vis);
        imshow("uline", uline_mask);
        return uhist_vis;
    }
    Mat extract_line(Mat umap, int max_disp)
    {
        Mat uline_mask = Mat::zeros(max_disp, image_size.width, CV_8UC3);
        uint8_t *p;
        int i, j, k, start, end;
        int threshold = max_disp * 0.1;
        int accumulate_threshold = 0;
        /*
        Note: for Matrix and Point Coordinate System
        0/0---column--->      0/0---X--->
        |                     |
        row     Mat           Y   Point
        |                     |
        v                     v
        */
        for (i = 1; i < umap.rows; ++i) // we skip disparity 0 which is corrupted part
        {
            p = umap.ptr<uint8_t>(i);
            accumulate_threshold = 0;
            start = end = -1;
            for (j = 0; j < umap.cols; ++j)
            {
                if (p[j] >= threshold)
                {
                    accumulate_threshold += p[j];
                    if (start == -1) start = j;

                }
                else if (accumulate_threshold > 2 * max_disp)
                {
                    end = j;
                    line(uline_mask, Point(start, i), Point(end, i), Scalar(0, 255, 0), 1, 8);
                    start = end = -1;
                    accumulate_threshold = 0;
                }
                else 
                {
                    start = end = -1;
                    accumulate_threshold = 0;
                }
                // if(p[j] == 255) cout << unsigned(p[j]) << " ";
            }
        }
        return uline_mask;
    }
}