#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <queue>
#include <utility>

using namespace cv;
using namespace std;

namespace umap_utility
{
    struct ellipse_desc {
        int u1; // left most
        int u2; // right most
        double d1; // max disp 
        double d2; // mean disp
        double GYe;
        Mat BPe;
        Mat BSe;
        Mat ESe;
        Mat GSe; 
        Mat GPe;
    };
    void add_neighbor(int i, int j, int dr, int dc, int maxr, int maxcol, Mat tmp_uline, Mat connected_component, queue< pair<int,int> >& neighbors, int mark);
    vector<ellipse_desc> calculate_udisparity(Mat disp_img, int max_disp, Size image_size, int& obj_count);
}
