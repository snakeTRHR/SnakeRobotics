#include<iostream>
#include"/usr/include/eigen3/Eigen/Dense"

using namespace std;
using namespace Eigen;
int main() {
  Eigen::Matrix<double, 3, 3> A;  // 実数行列
  A << 1,2,3,4,5,6,7,8,9;  // Aの行列要素を代入

  Eigen::EigenSolver< Eigen::Matrix<double, 3, 3> > s(A);
  std::cout << "固有値\n" << s.eigenvalues() << std::endl;
  std::cout << "固有ベクトル\n" << s.eigenvectors().real() << std::endl;
  Eigen::Matrix<double, 3, 3> ans = s.eigenvectors().real();
  cout << ans << endl;
  return 0;
}

