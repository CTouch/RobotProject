#include "Utils.h"

void forward(LearnPoint &point)
{
    Eigen::Matrix<double, 3, 1> local_p;
    Eigen::Matrix<double, 6, 1> angle;
    Eigen::Matrix<double, 6, 1> global_p;

    local_p << 0, 0, 0;
    angle << LIN2DEG(point.joint[0]), LIN2DEG(point.joint[1]), LIN2DEG(point.joint[2]), LIN2DEG(point.joint[3]), LIN2DEG(point.joint[4]), LIN2DEG(point.joint[5]);

    global_p = local2global(local_p, angle);

    for (int i = 0; i < 6; i++)
    {
        point.pos[i] = global_p(i, 0);
    }
}

void inverse(LearnPoint &point, LearnPoint &init)
{
    Eigen::Matrix<double, 6, 1> init_a;
    Eigen::Matrix<double, 6, 1> global_p;
    Eigen::Matrix<double, 6, 1> angle;

    global_p << point.pos[0], point.pos[1], point.pos[2], point.pos[3], point.pos[4], point.pos[5];

    init_a << LIN2DEG(init.joint[0]), LIN2DEG(init.joint[1]), LIN2DEG(init.joint[2]), LIN2DEG(init.joint[3]), LIN2DEG(init.joint[4]), LIN2DEG(init.joint[5]);

    angle = numerical_inverse(global_p, init_a);

    for (int i = 0; i < 6; i++)
    {
        point.joint[i] = DEG2LIN(angle(i, 0));
    }
}

Eigen::Matrix<double, 6, 1> numerical_inverse(Eigen::Matrix<double, 6, 1> global_pose, Eigen::Matrix<double, 6, 1> init_angle)
{
    Eigen::Matrix<double, 6, 1> motor_angle;
    Eigen::Matrix<double, 6, 1> pose, d_pose;
    Eigen::Matrix<double, 3, 1> local_position;
    Eigen::Matrix<double, 6, 6> J;
    local_position << 0, 0, 0;
    motor_angle << 0, 0, 0, 0, 0, 0;
    motor_angle = init_angle;
    pose = local2global(local_position, motor_angle); // 正运动学求解全局pose
    d_pose = global_pose - pose;                      // 计算d_pose并归一化
    for (int i = 3; i < 6; i++)
    {
        while (d_pose(i, 0) > M_PI)
            d_pose(i, 0) = d_pose(i, 0) - 2 * M_PI;
        while (d_pose(i, 0) < -M_PI)
            d_pose(i, 0) = d_pose(i, 0) + 2 * M_PI;
    }

    int num = 0;
    while (d_pose.norm() > 0.1)
    {
        // cout << "----------iter_time: " << num << "----------" << endl;
        // cout << "global_p: " << global_pose.transpose() << endl;
        // cout << "pose : " << pose.transpose() << endl;
        // cout << "d_norm:\t" << d_pose.norm() << endl;

        J = jacobian(local_position, motor_angle);
        // cout << "Jacobian: " << endl;
        // cout << J << endl << endl;

        motor_angle = motor_angle + (J.inverse() * d_pose);
        for (int i = 0; i < 6; i++)
        {
            while (motor_angle(i, 0) > 180)
                motor_angle(i, 0) = motor_angle(i, 0) - 360;
            while (motor_angle(i, 0) < -180)
                motor_angle(i, 0) = motor_angle(i, 0) + 360;
        }
        pose = local2global(local_position, motor_angle);
        num++;
        // 更新d_pose并做归一化
        d_pose = global_pose - pose;
        for (int i = 3; i < 6; i++)
        {
            while (d_pose(i, 0) > M_PI)
                d_pose(i, 0) = d_pose(i, 0) - 2 * M_PI;
            while (d_pose(i, 0) < -M_PI)
                d_pose(i, 0) = d_pose(i, 0) + 2 * M_PI;
        }
    }
    return motor_angle;
}

Eigen::Matrix<double, 6, 6> jacobian(Eigen::Matrix<double, 3, 1> local_position, Eigen::Matrix<double, 6, 1> angle)
{
    Eigen::Matrix<double, 6, 6> J;
    J.setZero(6, 6);
    double delta = 0.01;
    Eigen::Matrix<double, 6, 1> pose;

    pose = local2global(local_position, angle);

    Eigen::Matrix<double, 6, 1> d_angle;
    Eigen::Matrix<double, 6, 1> d_pose, delta_pose;
    for (int i = 0; i < 6; i++)
    {
        d_angle = angle;
        d_angle(i, 0) = d_angle(i, 0) + delta;

        d_pose = local2global(local_position, d_angle);
        delta_pose = d_pose - pose;

        for (int j = 3; j < 6; j++)
        {
            while (delta_pose(j, 0) < -M_PI)
                delta_pose(j, 0) = delta_pose(j, 0) + 2 * M_PI;
            while (delta_pose(j, 0) > M_PI)
                delta_pose(j, 0) = delta_pose(j, 0) - 2 * M_PI;
        }
        for (int j = 0; j < 6; j++)
        {
            J(j, i) = delta_pose(j, 0) / delta;
        }
    }
    return J;
}

Eigen::Matrix<double, 6, 1> local2global(Eigen::Matrix<double, 3, 1> local_position, Eigen::Matrix<double, 6, 1> motor_angle)
{

    Eigen::Matrix<double, 6, 1> a, alpha, d, theta, delta_theta;
    a << 0, 180, 27, 0, 0, 0;
    alpha << 90, 0, 90, 90, 90, 0;
    d << 253, 0, 0, 182.96, 0, 61.85;
    theta << 90, 90, 0, 180, 90, 0;
    local_position << 0, 0, 0;

    delta_theta = motor_angle;
    theta = theta + delta_theta;
    alpha = alpha * M_PI / 180;
    theta = theta * M_PI / 180;

    Eigen::Matrix<double, 4, 4> T[6];

    for (int i = 0; i < 6; i++)
    {
        T[i] << cos(theta(i, 0)), -cos(alpha(i, 0)) * sin(theta(i, 0)), sin(alpha(i, 0)) * sin(theta(i, 0)), a(i, 0) * cos(theta(i, 0)),
            sin(theta(i, 0)), cos(alpha(i, 0)) * cos(theta(i, 0)), -sin(alpha(i, 0)) * cos(theta(i, 0)), a(i, 0) * sin(theta(i, 0)),
            0, sin(alpha(i, 0)), cos(alpha(i, 0)), d(i, 0),
            0, 0, 0, 1;
    }
    Eigen::Matrix<double, 4, 4> T06, T67, T07;

    T06 = T[0] * T[1] * T[2] * T[3] * T[4] * T[5];
    T67.setIdentity(4, 4);
    for (int i = 0; i < 3; i++)
    {
        T67(i, 3) = local_position(i, 0);
    }
    T07 = T06 * T67;

    Eigen::Matrix<double, 6, 1> global_pose;
    global_pose(0, 0) = T07(0, 3);
    global_pose(1, 0) = T07(1, 3);
    global_pose(2, 0) = T07(2, 3);
    Eigen::Matrix<double, 3, 3> pose;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            pose(i, j) = T07(i, j);
            if (fabs(pose(i, j)) < 1e-6)
                pose(i, j) = 0;
        }
    }
    Eigen::Matrix<double, 3, 1> temp_pose;
    temp_pose = tr2RPY(pose);

    global_pose(3, 0) = temp_pose(0, 0);
    global_pose(4, 0) = temp_pose(1, 0);
    global_pose(5, 0) = temp_pose(2, 0);

    return global_pose;
}

Eigen::Matrix<double, 3, 1> tr2RPY(Eigen::Matrix<double, 3, 3> pose)
{
    double R = atan2(pose(2, 1), pose(2, 2));
    double Y = atan2(pose(1, 0), pose(0, 0));
    double P = atan2(-pose(2, 0), sqrt(pose(2, 1) * pose(2, 1) + pose(2, 2) * pose(2, 2)));
    Eigen::Matrix<double, 3, 1> RPY;
    RPY << R, P, Y;
    for (int i = 0; i < 3; i++)
    {
        while (RPY(i, 0) < -M_PI)
            RPY(i, 0) = RPY(i, 0) + 2 * M_PI;
        while (RPY(i, 0) > M_PI)
            RPY(i, 0) = RPY(i, 0) - 2 * M_PI;
    }
    return RPY;
}
