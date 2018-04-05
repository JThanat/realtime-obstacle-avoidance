#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace umap_utility
{
    cv::Mat calculate_udisparity(cv::Mat disp_img, int max_disp, cv::Size image_size);
    cv::Mat extract_line(cv::Mat umap, int max_disp); 
}
