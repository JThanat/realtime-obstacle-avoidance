#include "umap_util.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <string>
#include <queue>
#include <utility>
#include <vector>

using namespace cv;
using namespace std;

namespace umap_utility
{
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

    vector<ellipse_desc> calculate_udisparity(Mat disp_img, int max_disp, Size image_size, int& ojb_count)
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
        vector<ellipse_desc> ellipse_desc_objects(20);

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
        cout << "Calc UMap Time elapsed: " << t * 1000 / getTickFrequency() << "ms" << endl;
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
        cout << "Finding Line Time elapsed: " << t * 1000 / getTickFrequency() << "ms" << endl;

        // cvtColor(uline_mask, uline_mask, CV_GRAY2BGR);

        // connect line
        t = getTickCount();
        int mark = 1;
        int diff_inc = 1;
        int object_count = 0;
        int u,v;
        int maxd, maxc, mind, minc; // max-min disp and max-min col
        int sumd; // sum of disp
        int component_count;
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
                    component_count = 0;
                    sumd = 0;
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
                        component_count++;
                        sumd += idx.first;
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
                            add_neighbor(idx.first, idx.second, 5, 10, tmp_uline.rows, tmp_uline.cols, tmp_uline, connected_component, neighbors, mark);
                        }
                        else 
                        {
                            add_neighbor(idx.first, idx.second, 2, 4, tmp_uline.rows, tmp_uline.cols, tmp_uline, connected_component, neighbors, mark);
                        }
                    }
                    // cout << "object " << object_count << " " << minc << "," << maxc << " " << mind << "," << maxd << endl;
                    if (object_count <= ellipse_desc_objects.size()) 
                    {
                        ellipse_desc_objects[object_count - 1].u1 = minc;
                        ellipse_desc_objects[object_count - 1].u2 = maxc;
                        ellipse_desc_objects[object_count - 1].d1 = maxd;
                        ellipse_desc_objects[object_count - 1].d2 = sumd/component_count;
                        cout << "ellipse_desc:" << object_count - 1 << " " << ellipse_desc_objects[object_count - 1].u1 << " " << ellipse_desc_objects[object_count - 1].u2 << " " << ellipse_desc_objects[object_count - 1].d1 << " " << ellipse_desc_objects[object_count - 1].d2 << " " << (mind+maxd)/2 << endl;
                    }
                    // line(group_rect, Point(minc, sumd/component_count), Point(maxc,sumd/component_count), Scalar(0,255,0));
                    rectangle(group_rect, Point(minc, maxd), Point(maxc, mind), Scalar(255,0,0));
                    mark += diff_inc;
                }
            }
        }
        // for (i = 0 ; i < 4 ; i++)
        //     line(group_rect, Point(0, i*max_disp/4), Point(group_rect.cols, i*max_disp/4), Scalar(0,0,255));
        // cvtColor(uline_mask,uline_mask, CV_GRAY2BGR);
        // for (i = 0 ; i < group_rect.rows ; i=i+10)
        //     line(uline_mask, Point(0, i), Point(group_rect.cols, i), Scalar(0,255,0));

        cout << "Number of object " << object_count << endl;
        ojb_count = object_count;

        t = getTickCount() - t;
        cout << "Connect the Line Time elapsed: " << t * 1000 / getTickFrequency() << "ms" << endl;
        imwrite("./uhist.jpg", uhist_vis);
        imwrite("./group_rect.jpg", group_rect);
        imwrite("./uline.jpg", uline_mask);

        // return u1, u2, d1, d2 of each object
        

        return ellipse_desc_objects;
    }
}