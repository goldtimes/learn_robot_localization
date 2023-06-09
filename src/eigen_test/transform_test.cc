#include <Eigen/Dense>
#include <iostream>

int main(int argc, char** argv) {
  Eigen::Matrix4d Ti;
  Eigen::Matrix4d Tj;
  Eigen::Quaterniond qi;
  qi.w() = 1.0;
  qi.x() = 0.0;
  qi.y() = 0.0;
  qi.z() = 0.0;
  qi.normalized();
  Eigen::Quaterniond qj;
  qj.w() = 0.707;
  qj.x() = 0.707;
  qj.y() = 0.0;
  qj.z() = 0.0;
  qj.normalized();
  Eigen::Matrix3d rotate_i = qi.toRotationMatrix();
  Eigen::Matrix3d rotate_j = qj.toRotationMatrix();

  auto r_vec_i = rotate_i.eulerAngles(0, 1, 2);
  std::cout << "r_vec_i: " << r_vec_i << std::endl;
  auto r_vec_j = rotate_j.eulerAngles(0, 1, 2);
  std::cout << "r_vec_j: " << r_vec_j << std::endl;
  Eigen::VectorXd vi, vj;
  vi.resize(4);
  vj.resize(4);
  vi << 1.0, 0.0, 0.0, 1.0;
  vj << 2.0, 0.0, 0.0, 1.0;
  Ti.block<3, 3>(0, 0) = rotate_i;
  Ti.block<4, 1>(0, 3) = vi;
  Tj.block<3, 3>(0, 0) = rotate_j;
  Tj.block<4, 1>(0, 3) = vj;

  std::cout << "Ti: " << Ti << std::endl;
  std::cout << "Tj: " << Tj << std::endl;
  std::cout << "Ri * Rj.transport: "
            << (rotate_i * rotate_j.transpose()).eulerAngles(0, 1, 2)
            << std::endl;
}