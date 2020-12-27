#include <iostream>

#include <Eigen/Core>
//稠密矩阵的代数运算（逆、特征值等）
#include <Eigen/Dense>
#include <math.h>

class Forward
{
private:
    Eigen::Matrix<double,6,1> a;
    Eigen::Matrix<double,6,1> d;
    Eigen::Matrix<double,6,1> alpha;
    Eigen::Matrix<double,6,1> theta;
    Eigen::Matrix<
public:
    Forward(/* args */);
    ~Forward();
};

Forward::Forward(/* args */)
{
    a << 0, 180, 27, 0, 0, 0;
    d << 253, 0, 0, 182.96, 0, 0;
    alpha << M_PI_2, 0, M_PI_2, M_PI_2, M_PI_2, 0;
    theta << 0, M_PI_2, 0, M_PI_2, M_PI_2, 0;

}

Forward::~Forward()
{
}
