#include <iostream>
#include <fstream>
#include <unistd.h>
#include <pangolin/pangolin.h>
#include <sophus/se3.h>

using namespace Sophus;
using namespace std;

string groundtruth_file = "/home/shuchun/下载/slambook2-master/ch4/example/groundtruth.txt";//真实的轨迹
string estimated_file = "/home/shuchun/下载/slambook2-master/ch4/example/estimated.txt";  //估计的轨迹

typedef vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> TrajectoryType;

void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti);

TrajectoryType ReadTrajectory(const string &path);

int main(int argc, char **argv) {
  TrajectoryType groundtruth = ReadTrajectory(groundtruth_file);
  TrajectoryType estimated = ReadTrajectory(estimated_file);
  assert(!groundtruth.empty() && !estimated.empty());  //断言,检测程序参数的合法性,若不正常,报错且终止程序
  assert(groundtruth.size() == estimated.size());

  // compute rmse(均方根误差)
  double rmse = 0;
  for (size_t i = 0; i < estimated.size(); i++) {
    Sophus::SE3 p1 = estimated[i], p2 = groundtruth[i];
    //cout << p1 << endl ;
   // cout<< p2 << endl;
   double error = (p2.inverse() * p1).log().norm();
   // cout << "error=" << error << endl;
    rmse += error * error;

  }
  rmse = rmse / double(estimated.size());
  cout << estimated.size() << endl;  //estimated.size() 表示的是estimated.txt数据的个数
  cout << "rmse=" << rmse << endl;
  rmse = sqrt(rmse);
  cout << "RMSE = " << rmse << endl;  //均方根误差

  DrawTrajectory(groundtruth, estimated);
  return 0;
}

TrajectoryType ReadTrajectory(const string &path) {
  ifstream fin(path); //打开文件所在路径
  TrajectoryType trajectory;
  if (!fin) {
    cerr << "trajectory " << path << " not found." << endl;  //常用于输出错误的信息
    return trajectory;
  }

  while (!fin.eof()) {
    double time, tx, ty, tz, qx, qy, qz, qw;
    fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
    Sophus::SE3 p1(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz));
    trajectory.push_back(p1);
  }
  return trajectory;
}

void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti) {
  // create pangolin window and plot the trajectory
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
      pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
  );

  pangolin::View &d_cam = pangolin::CreateDisplay()
      .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
      .SetHandler(new pangolin::Handler3D(s_cam));


  while (pangolin::ShouldQuit() == false) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glLineWidth(2);
    for (size_t i = 0; i < gt.size() - 1; i++) {
      glColor3f(0.0f, 0.0f, 1.0f);  // blue for ground truth
      glBegin(GL_LINES);
      auto p1 = gt[i], p2 = gt[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }

    for (size_t i = 0; i < esti.size() - 1; i++)
    {
      glColor3f(1.0f, 0.0f, 0.0f);  // red for estimated
      glBegin(GL_LINES);
      auto p1 = esti[i], p2 = esti[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }
    pangolin::FinishFrame();
    usleep(5000);   // sleep 5 ms
  }

}
