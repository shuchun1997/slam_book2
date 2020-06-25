#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <pangolin/pangolin.h>
#include <unistd.h>

using namespace std;
using namespace Eigen;

// 文件路径
string left_file = "/home/shuchun/下载/slambook2-master/ch5/stereo/left.png";
string right_file = "/home/shuchun/下载/slambook2-master/ch5/stereo/right.png";

// 在pangolin中画图，已写好，无需调整
void showPointCloud(
        const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud); //函数定义

int main(int argc, char **argv)
{

    // 内参
    double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
    // 基线
    double b = 0.573;

    // 读取图像
    cv::Mat left = cv::imread(left_file, 0);
    cv::Mat right = cv::imread(right_file, 0);
    //立体匹配算法SGBM
    //minDisparity：最小视差，默认为0。此参数决定左图中的像素点在右图匹配搜索的起点，int类型；
    //numDisparities：视差搜索范围长度，其值必须为16的整数倍。最大视差 maxDisparity = minDisparity + numDisparities -1；
    //blockSize：SAD代价计算窗口大小，默认为5。窗口大小为奇数，一般在3*3 到21*21之间；
    //P1、P2：能量函数参数,P1是相邻像素点视差增/减1时的惩罚系数；P2是相邻像素点视差变化值大于1时的惩罚系数。P2必须大于P1.需要指出,在动态规划时,P1和P2都是常数。
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
        0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);    // 神奇的参数
    cv::Mat disparity_sgbm, disparity;
    sgbm->compute(left, right, disparity_sgbm);  // (->指针的指向运算符)  由左右视图按照SGBM匹配方式计算得到视差图
    disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f);   //将16位符号整形的视差Mat转换为32位浮点型Mat

    // 生成点云
    vector<Vector4d, Eigen::aligned_allocator<Vector4d>> pointcloud;  //定义4维形式的点云向量容器

    // 如果你的机器慢，请把后面的v++和u++改成v+=2, u+=2
    for (int v = 0; v < left.rows; v++)  //遍历左视图
        for (int u = 0; u < left.cols; u++)
        {
            if (disparity.at<float>(v, u) <= 0.0 || disparity.at<float>(v, u) >= 96.0) continue;

            Vector4d point(0, 0, 0, left.at<uchar>(v, u) / 255.0); //前三维为xyz,第四维为颜色,第4维为归一化后的强度值

            // 根据双目模型计算 point 的位置
            double x = (u - cx) / fx;  //该公式计算的是归一化在相机Z=1平面的相机坐标
            double y = (v - cy) / fy;
            double depth = fx * b / (disparity.at<float>(v, u));  //由视差,双目的基计算像素点对应的实际距离（深度信息）
            point[0] = x * depth; //由深度信息获取真实相机坐标系下的Xc
            point[1] = y * depth; //由深度信息获取真实相机坐标系下的Yc
            point[2] = depth;     //相机坐标系下的Zc

            pointcloud.push_back(point);   //获得的是相机坐标系下的点云位置
        }

    cv::imshow("disparity", disparity / 96.0);   //把视差值限定在0-96
    cv::waitKey(0);
    // 画出点云
    showPointCloud(pointcloud);
    return 0;
}

void showPointCloud(const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud)  //函数实现
{

    if (pointcloud.empty())   //确保点云容器非空
    {
        cerr << "Point cloud is empty!" << endl;
        return;
    }

    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000), //相机参数
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)     //观测视角
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p: pointcloud) {
            glColor3f(p[3], p[3], p[3]);  //点的颜色
            glVertex3d(p[0], p[1], p[2]);  //点的相机坐标
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return;
}