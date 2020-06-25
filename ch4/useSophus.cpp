#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.h"

using namespace std;
using namespace Eigen;

/// 本程序演示sophus的基本用法

int main(int argc, char **argv) {

  // 第一种方法表示沿Z轴转90度的旋转矩阵
  Matrix3d R = AngleAxisd(M_PI / 2, Vector3d(0, 0, 1)).toRotationMatrix();
  cout << R << endl;


  //第二种方法表示沿z轴旋转90的旋转矩阵
  /*
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity(); //矩阵R初始化成单位阵
  AngleAxisd rotation_vector ( M_PI/2, Vector3d ( 0,0,1 ) );
  R=rotation_vector.toRotationMatrix();
  cout << R << endl;
   */

  // 或者四元数
  Quaterniond q(R);
  cout << q.coeffs().transpose() << endl;  //获取四元数
  Sophus::SO3 SO3_R(R);              // Sophus::SO3可以直接从旋转矩阵构造
  Sophus::SO3 SO3_q(q);              // 也可以通过四元数构造
  // 二者是等价的
  cout << "SO(3) from matrix:\n" << SO3_R.matrix() << endl;  //输出李群
  cout << "SO(3) from quaternion:\n" << SO3_q.matrix() << endl;
  cout << "they are equal" << endl;

  // 使用对数映射获得它的李代数
  Vector3d so3 = SO3_R.log();
  cout << "so3 = " << so3.transpose() << endl;
  // hat 为向量到反对称矩阵
  cout << "so3 hat=\n" << Sophus::SO3::hat(so3) << endl;
  // 相对的，vee为反对称到向量
  cout << "so3 hat vee= " << Sophus::SO3::vee(Sophus::SO3::hat(so3)).transpose() << endl;

  // 增量扰动模型的更新
  Vector3d update_so3(1e-4, 0, 0); //假设更新量为这么多
  Sophus::SO3 SO3_updated = Sophus::SO3::exp(update_so3) * SO3_R;
  cout << "SO3 updated = \n" << SO3_updated.matrix() << endl;  //输出exp(update_so3)*so3_R

  cout << "*******************************" << endl;
  // 对SE(3)操作大同小异
  Vector3d t(1, 0, 0);           // 沿X轴平移1
  Sophus::SE3 SE3_Rt(R, t);           // 从R,t构造SE(3)
  Sophus::SE3 SE3_qt(q, t);            // 从q,t构造SE(3)
  cout << "SE3 from R,t= \n" << SE3_Rt.matrix() << endl; //SE3实际是4*4的
  cout << "SE3 from q,t= \n" << SE3_qt.matrix() << endl;
  // 李代数se(3) 是一个六维向量，方便起见先typedef一下
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  Vector6d se3 = SE3_Rt.log();  //李代数se(3) 是一个6维向量为李群SE3 的对数映射
  cout << "se3 = " << se3.transpose() << endl;
  // 观察输出，会发现在Sophus中，se(3)的平移在前，旋转在后.
  // 同样的，有hat和vee两个算符
  cout << "se3 hat = \n" << Sophus::SE3::hat(se3) << endl;  // hat 为向量到反对称矩阵,相当于　^　运算。
  cout << "se3 hat vee = " << Sophus::SE3::vee(Sophus::SE3::hat(se3)).transpose() << endl;  //相对的,vee为反对称矩阵到向量相当于下尖尖运算

  // 最后，演示一下更新
  Vector6d update_se3; //更新量
  update_se3.setZero();
  update_se3(0, 0) = 1e-4d;
  Sophus::SE3 SE3_updated = Sophus::SE3::exp(update_se3) * SE3_Rt;  //指数映射
  cout << "SE3 updated = " << endl << SE3_updated.matrix() << endl;

  return 0;
}