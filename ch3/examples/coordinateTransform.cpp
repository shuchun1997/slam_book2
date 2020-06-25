#include <iostream>
#include <vector>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
  Quaterniond q1(0.35, 0.2, 0.3, 0.1), q2(-0.5, 0.4, -0.1, 0.2);//Quaterniond 表示四元数 一个实部三个虚部,w为实部
  cout << "第一个四元数q1:" <<q1.coeffs().transpose() << endl;      //.transpose()表示对四元数进行转置一下
  cout << "第二个四元数q2:" << q2.coeffs().transpose() << endl;
  q1.normalize();
  q2.normalize();
  Vector3d t1(0.3, 0.1, 0.1), t2(-0.1, 0.5, 0.3);   //欧拉角
  Vector3d p1(0.5, 0, 0.2);
    cout << p1 << endl;
  Isometry3d T1w(q1), T2w(q2);   //Isometry3d = Eigen::Isometry3d 表示欧式变换矩阵
  T1w.pretranslate(t1);
  //VectorXd p5 = T1w.pretranslate(t1);
  //cout << p5 << endl;
  T2w.pretranslate(t2);

  Vector3d p2 = T2w * T1w.inverse() * p1;  //
  cout << endl << p2.transpose() << endl;
  return 0;
}