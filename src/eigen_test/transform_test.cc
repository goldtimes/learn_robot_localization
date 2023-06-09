#include <Eigen/Dense>
#include <iomanip>
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

  // std::cout << "Ti: " << Ti << std::endl;
  // std::cout << "Tj: " << Tj << std::endl;
  // std::cout << "Ri * Rj.transport: "
  //           << (rotate_i * rotate_j.transpose()).eulerAngles(0, 1, 2)
  //           << std::endl;
  Eigen::Matrix3d Ri =
      Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
  Eigen::Matrix3d Rj =
      Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
  std::cout << "Ri: \t" << std::fixed << std::setprecision(3) << Ri
            << std::endl;
  std::cout << "Ri: \t" << std::fixed << std::setprecision(3) << Rj
            << std::endl;
  std::cout << "Ri*Rj.transport(): " << std::fixed << std::setprecision(3)
            << Ri * Rj.transpose() << std::endl;
  // 旋转差 居然是一样的Rj.transport() * Ri  = Ri *Rj.transport()
  std::cout << "Rj.transport() * Ri " << std::fixed << std::setprecision(3)
            << Rj.transpose() * Ri << std::endl;
  std::cout << "Ri *Rj.transport()"
            << (Ri * Rj.transpose()).eulerAngles(0, 1, 2) << std::endl;
  std::cout << "Rj.transport() * Ri.eular(): "
            << (Rj.transpose() * Ri).eulerAngles(0, 1, 2) << std::endl;
}