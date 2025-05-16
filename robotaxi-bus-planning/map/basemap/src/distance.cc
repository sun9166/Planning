#include <iostream> 
#include <opencv2/core/core.hpp> 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
  
using namespace std;


namespace basemap_shm_util {

static const int DIST_SHIFT = 16;
static const int INIT_DIST0 = 0; // set BORDER of mat as obstacle
static const int DIST_MAX   = ((0x7fffffff) >> 2);
#define  CV_FLT_TO_FIX(x,n)  cvRound((x)*(1<<(n)))

void NearestShow(cv::Mat& dstImage, cv::Mat& nearest)
{
    cv::Size size = dstImage.size();
    for (int i = 0; i < size.height; i++) {
        for (int j = 0; j < size.width; j++) {
            int distance = (int)dstImage.at<float>(i, j);
            int point_x = (int)nearest.at<cv::Vec2w>(i, j)[0];
            int point_y = (int)nearest.at<cv::Vec2w>(i, j)[1];
            std::cout << distance << "=(" << point_x << "," << point_y << ") ";
        }
        std::cout << std::endl;
    }
}

void initTopBottom(cv::Mat& temp, int border)
{
    cv::Size size = temp.size();
    for( int i = 0; i < border; i++ )
    {
        int* ttop = temp.ptr<int>(i);
        int* tbottom = temp.ptr<int>(size.height - i - 1);

        for( int j = 0; j < size.width; j++ )
        {
            ttop[j] = INIT_DIST0;
            tbottom[j] = INIT_DIST0;
        }
    }
}

void getDistanceTransformMask( int maskType, float *metrics )
{
    CV_Assert( metrics != 0 );

    // std::cout << "maskType is " << maskType << std::endl;

    switch (maskType)
    {
    case 30:
        metrics[0] = 1.0f;
        metrics[1] = 1.0f;
        break;

    case 31:
        metrics[0] = 1.0f;
        metrics[1] = 2.0f;
        break;

    case 32:
        metrics[0] = 0.955f;
        metrics[1] = 1.3693f;
        break;

    case 50:
        metrics[0] = 1.0f;
        metrics[1] = 1.0f;
        metrics[2] = 2.0f;
        break;

    case 51:
        metrics[0] = 1.0f;
        metrics[1] = 2.0f;
        metrics[2] = 3.0f;
        break;

    case 52:
        metrics[0] = 1.0f;
        metrics[1] = 1.4f;
        metrics[2] = 2.1969f;
        break;
    default:
        CV_Error(CV_StsBadArg, "Unknown metric type");
    }
}

void distanceTransform_3x3( const cv::Mat& _src, cv::Mat& _temp, 
        cv::Mat& _dist, cv::Mat& _nearest, const float* metrics)
{
    const int BORDER = 1;
    int i, j;
    const unsigned int HV_DIST = CV_FLT_TO_FIX( metrics[0], DIST_SHIFT );
    const unsigned int DIAG_DIST = CV_FLT_TO_FIX( metrics[1], DIST_SHIFT );
    const float scale = 1.f/(1 << DIST_SHIFT);

    const uchar* src = _src.ptr();
    int* temp = _temp.ptr<int>();
    float* dist = _dist.ptr<float>();
    int srcstep = (int)(_src.step/sizeof(src[0]));
    int step = (int)(_temp.step/sizeof(temp[0]));
    int dststep = (int)(_dist.step/sizeof(dist[0]));
    cv::Size size = _src.size();

    initTopBottom( _temp, BORDER );

    // std::cout << "DIAG_DIST=" << DIAG_DIST << ", HV_DIST=" << HV_DIST << std::endl;

    // forward pass
    for( i = 0; i < size.height; i++ )
    {
        const uchar* s = src + i*srcstep;
        unsigned int* tmp = (unsigned int*)(temp + (i+BORDER)*step) + BORDER;

        for( j = 0; j < BORDER; j++ )
            tmp[-j-1] = tmp[size.width + j] = INIT_DIST0;

        // set top and bottom border as obstacle
        if ((0==i) || ((size.height-1)==i)) {
            continue;
        }

        for( j = 0; j < size.width; j++ )
        {
            // set left and right border as obstacle
            if ((0==j) || ((size.width-1)==j)) {
                tmp[j] = 0;
                continue;
            }

            // std::cout << "forward " << i << ", " << j << std::endl;
            // std::cout << _temp << std::endl << std::endl;
            if( !s[j] )
                tmp[j] = 0;
            else
            {
                int nearest_x;
                int nearest_y;
                nearest_x = i-1;
                nearest_y = j-1;
                unsigned int t0 = tmp[j-step-1] + DIAG_DIST;
                unsigned int t = tmp[j-step] + HV_DIST;
                if( t0 > t ) {
                    t0 = t;  
                    nearest_x = i-1;
                    nearest_y = j;
                }
                t = tmp[j-step+1] + DIAG_DIST;
                if( t0 > t ) {
                    t0 = t;
                    nearest_x = i-1;
                    nearest_y = j+1;
                }
                t = tmp[j-1] + HV_DIST;
                if( t0 > t ) {
                    t0 = t;
                    nearest_x = i;
                    nearest_y = j-1;
                }
                tmp[j] = t0;
                if ((0 == _nearest.at<cv::Vec2w>(nearest_x, nearest_y)[0]) && 
                        (0 == _nearest.at<cv::Vec2w>(nearest_x, nearest_y)[1])) {
                    // this is the boundary point
                    _nearest.at<cv::Vec2w>(i, j)[0] = nearest_x;
                    _nearest.at<cv::Vec2w>(i, j)[1] = nearest_y;
                } else {
                    // record boundary point
                    _nearest.at<cv::Vec2w>(i, j)[0] = _nearest.at<cv::Vec2w>(nearest_x, nearest_y)[0];
                    _nearest.at<cv::Vec2w>(i, j)[1] = _nearest.at<cv::Vec2w>(nearest_x, nearest_y)[1];
                }
            }
            // std::cout << "forward show distance and nearest point: (" << i << ", " << j << ")" << std::endl;
            // NearestShow(_dist, _nearest);
        }
    }

    // std::cout << "forward final" << std::endl;
    // std::cout << _temp << std::endl << std::endl;
    // NearestShow(_nearest);

    // backward pass
    for( i = size.height - 1; i >= 0; i-- )
    {
        float* d = (float*)(dist + i*dststep);
        unsigned int* tmp = (unsigned int*)(temp + (i+BORDER)*step) + BORDER;

        for( j = size.width - 1; j >= 0; j-- )
        {
            int nearest_x;
            int nearest_y;
            unsigned int t0 = tmp[j];
            bool useBack = false;
            if( t0 > HV_DIST )
            {
                // std::cout << "backward " << i << ", " << j << std::endl;
                // std::cout << _dist << std::endl << std::endl;
                unsigned int t = tmp[j+step+1] + DIAG_DIST;
                nearest_x = i+1;
                nearest_y = j+1;
                if( t0 > t ) {
                    // std::cout << "i+1, j+1" << std::endl;
                    t0 = t;
                    useBack = true;
                }
                t = tmp[j+step] + HV_DIST;
                if( t0 > t ) {
                    // std::cout << "i+1, j" << std::endl;
                    t0 = t;
                    useBack = true;
                    nearest_x = i+1;
                    nearest_y = j;
                }
                t = tmp[j+step-1] + DIAG_DIST;
                if( t0 > t ) {
                    // std::cout << "i+1, j-1" << std::endl;
                    t0 = t;
                    useBack = true;
                    nearest_x = i+1;
                    nearest_y = j-1;
                }
                t = tmp[j+1] + HV_DIST;
                if( t0 > t ) {
                    // std::cout << "i, j+1" << std::endl;
                    t0 = t;
                    useBack = true;
                    nearest_x = i;
                    nearest_y = j+1;
                }
                tmp[j] = t0;
                if (useBack) {
                    if ((0 == _nearest.at<cv::Vec2w>(nearest_x, nearest_y)[0]) && 
                            (0 == _nearest.at<cv::Vec2w>(nearest_x, nearest_y)[1])) {
                        // use new
                        // std::cout << "use new: " << nearest_x << ", " << nearest_x << std::endl;
                       _nearest.at<cv::Vec2w>(i, j)[0] = nearest_x;
                        _nearest.at<cv::Vec2w>(i, j)[1] = nearest_y;
                    } else {
                        // record boundary point
                        // use old
                        _nearest.at<cv::Vec2w>(i, j)[0] = _nearest.at<cv::Vec2w>(nearest_x, nearest_y)[0];
                        _nearest.at<cv::Vec2w>(i, j)[1] = _nearest.at<cv::Vec2w>(nearest_x, nearest_y)[1];
                    }
                }
            }
            t0 = (t0 > DIST_MAX) ? DIST_MAX : t0;
            d[j] = (float)(t0 * scale);
            // std::cout << "backward show distance and nearest point: (" << i << ", " << j << ")" << std::endl;
            // NearestShow(_dist, _nearest);
        }
    }
    // std::cout << "backward final" << std::endl;
    // std::cout << _temp << std::endl << std::endl;
    // std::cout << "nearst point" << std::endl;
    // std::cout << _nearest << std::endl << std::endl;
}

void distanceTransform1(cv::Mat _src, cv::Mat& _dst, cv::Mat& _nearest,
                            int distType, int maskSize, int labelType)
{
    int border = maskSize == CV_DIST_MASK_3 ? 1 : 2;
    cv::Size size = _src.size();
    cv::Mat temp( size.height + border*2, size.width + border*2, CV_32SC1 );

    float _mask[5] = {0};
    _dst.create( _src.size(), CV_32F);
    _nearest = cv::Mat::zeros(size.height, size.width, CV_8UC4);;

    getDistanceTransformMask( (distType == CV_DIST_C ? 0 :
        distType == CV_DIST_L1 ? 1 : 2) + maskSize*10, _mask );

    distanceTransform_3x3(_src, temp, _dst, _nearest, _mask);
}

}  // basemap_shm_util
