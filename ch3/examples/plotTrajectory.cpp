#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <unistd.h>

// 本例演示了如何画出一个预先存储的轨迹

using namespace std;
using namespace Eigen;


// path to trajectory file
string trajectory_file = "/home/shuchun/下载/slambook2-master/ch3/examples/trajectory.txt";

void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>>);   //DrawTrajectory 函数定义

int main(int argc, char **argv)
{
  vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses;
  ifstream fin(trajectory_file);//ifstream 该数据类型表示输入文件流，用于从文件读取信息。

  if (!fin)  //"!"是取非运算符
  {
    cout << "cannot find trajectory file at " << trajectory_file << endl;
    return 1;
  }

  while (!fin.eof())
  {
    double time, tx, ty, tz, qx, qy, qz, qw;  //指该位子的记录时间,t表示平移,q为旋转四元数,均是世界坐标系到相机坐标系的记录
    fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
    Isometry3d Twr(Quaterniond(qw, qx, qy, qz));
    Twr.pretranslate(Vector3d(tx, ty, tz));
    poses.push_back(Twr);
  }
  cout << "read total " << poses.size() << " pose entries" << endl;

  // draw trajectory in pangolin
  DrawTrajectory(poses);
  return 0;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses) {
  // create pangolin window and plot the trajectory
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);  //创建画图窗口
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
    pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
  );

  pangolin::View &d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));

  while (pangolin::ShouldQuit() == false) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);  //1.0f表示数据类型为float型,整数个数为1个,小数个数为0个;
    glLineWidth(3);   //设置线宽
    for (size_t i = 0; i < poses.size(); i++) {
      // 画每个位姿的三个坐标轴
      Vector3d Ow = poses[i].translation();
      Vector3d Xw = poses[i] * (0.1 * Vector3d(1, 0, 0));
      Vector3d Yw = poses[i] * (0.1 * Vector3d(0, 1, 0));
      Vector3d Zw = poses[i] * (0.1 * Vector3d(0, 0, 1));
      glBegin(GL_LINES);
      glColor3f(1.0, 0.0, 0.0);   //颜色设定为红色
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Xw[0], Xw[1], Xw[2]);
      glColor3f(0.0, 1.0, 0.0);   //颜色设定为绿色
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Yw[0], Yw[1], Yw[2]);
      glColor3f(0.0, 0.0, 1.0);   //颜色设定为蓝色
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Zw[0], Zw[1], Zw[2]);
      glEnd();
    }
    // 画出连线

    for (size_t i = 0; i < poses.size(); i++)
       {
        glColor3f(0.0, 0.0, 0.0);   //rgb参数全为零时,表示黑色.
        glBegin(GL_LINES);
        auto p1 = poses[i], p2 = poses[i + 1];
        glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
        glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
        glEnd();
      }

      pangolin::FinishFrame();
      usleep(5000);   // sleep 5 ms
    }
  }
