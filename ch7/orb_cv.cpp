#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>
#include <string>
using namespace std;
using namespace cv;
//程序读图部分改动过,注释部分为原来的读图程序

string first_file = "/home/shuchun/Pictures/1.png";
string second_file = "/home/shuchun/Pictures/2.png";

int main(int argc, char **argv) {
 /* if (argc != 3) {
    cout << "usage: feature_extraction img1 img2" << endl;
    return 1;
  }
 */
  //-- 读取图像
  /*
  Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
  Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);
   */

  Mat img_1 = imread(first_file, 0);
  Mat img_2 = imread(second_file, 0);
  assert(img_1.data != nullptr && img_2.data != nullptr);

  //-- 初始化
  std::vector<KeyPoint> keypoints_1, keypoints_2;  //存放关键点的容器
  Mat descriptors_1, descriptors_2;    //描述子
  Ptr<FeatureDetector> detector = ORB::create();  //括号里面可以指定关键点数量
  Ptr<DescriptorExtractor> descriptor = ORB::create();
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");


  //我们能通过刚定义的detector中的detect函数，将img中的像素进行分析处理，并将提取出的特征点存于keypoints容器中；
  // 之后再使用刚定义的descriptor中的compute函数,对每张img中keypoints所对应的每个像素点进行描述子的计算，并存于Mat类变量descriptor中。

  //-- 第一步:检测 Oriented FAST 角点位置
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  detector->detect(img_1, keypoints_1);
  detector->detect(img_2, keypoints_2);

  //-- 第二步:根据角点位置计算 BRIEF 描述子
  descriptor->compute(img_1, keypoints_1, descriptors_1);
  descriptor->compute(img_2, keypoints_2, descriptors_2);
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "extract ORB cost = " << time_used.count() << " seconds. " << endl;
  cout <<"The number of Keypoints_1="<< keypoints_1.size() << endl;
  //特征提取第一张图
  Mat outimg1;
  //drawKeypoints()函数原型参数:原图,原图关键点,带有关键点的输出图像,后面两个为默认值
  drawKeypoints(img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
  imshow("ORB features1", outimg1);

  //提取第二张图
  Mat outimg2;
  drawKeypoints(img_2, keypoints_2, outimg2, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
  imshow("ORB features2", outimg2);


  //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
  //创建一个匹配点数组，用于承接匹配出的DMatch，其实叫match_points_array更为贴切。matches类型为数组，元素类型为DMatch
  vector<DMatch> matches;
  t1 = chrono::steady_clock::now();
  matcher->match(descriptors_1, descriptors_2, matches);
  t2 = chrono::steady_clock::now();
  time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "match ORB cost = " << time_used.count() << " seconds. " << endl;

  //-- 第四步:匹配点对筛选
  // 计算最小距离和最大距离
  auto min_max = minmax_element(matches.begin(), matches.end(),
                                [](const DMatch &m1, const DMatch &m2) { return m1.distance < m2.distance; });
  double min_dist = min_max.first->distance;
  double max_dist = min_max.second->distance;

  printf("-- Max dist : %f \n", max_dist);
  printf("-- Min dist : %f \n", min_dist);

  //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
  std::vector<DMatch> good_matches;  //std::可以省略
  for (int i = 0; i < descriptors_1.rows; i++)
  {
    if (matches[i].distance <= max(2 * min_dist, 30.0))
    {
      good_matches.push_back(matches[i]);
    }
  }

  //这里调用drawMatches函数对两张图像img_1,img_2以及其之间的特征点配对进行连线与拼接,
  // 将左右两张图拼接成一张图并存入Mat类型对象img_match中。

  //-- 第五步:绘制匹配结果
  Mat img_match;
  Mat img_goodmatch;
  drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
  drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch);
  imshow("all matches", img_match);
  imshow("good matches", img_goodmatch);
  waitKey(0);

  return 0;
}
