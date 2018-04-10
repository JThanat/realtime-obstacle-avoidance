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
        int d1; // max disp 
        int d2; // mean disp
        Mat BPe;
        Mat BSe;
        float GSigmaE;
        float yaw_angle;
    };

    void add_neighbor(int i, int j, int dr, int dc, int maxr, int maxcol, Mat tmp_uline, Mat connected_component, queue< pair<int,int> >& neighbors, int mark);
    vector<ellipse_desc> calculate_udisparity(Mat disp_img, int max_disp, Size image_size);
}
