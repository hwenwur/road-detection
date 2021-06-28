#ifndef ROAD_DETECTOR_H_
#define ROAD_DETECTOR_H_

#include <iostream>
#include <vector>
#include <cmath>

namespace hb
{
    // 表示空间中的一个点
    typedef struct
    {
        float x;
        float y;
        float z;
        float intensity; // 强度
        int rgb;         // 颜色
    } Point3d;

    // 表示一个路沿的方程（直线）
    typedef struct
    {
        float k;        // 斜率
        float b;        // 截距
        float x_max;    // x 轴最大值，单位：m
        float x_min;    // x 轴最小值，单位：m
        bool available; // 是否有效
    } CurbInfo;

    // 一维卡尔曼滤波器
    // 本项目中，直线方程的 k, b 相互独立，
    // 为免于使用矩阵，先写一个一维的滤波器，再分别对 k, b 处理。
    class KalmanFilter
    {
    public:
        // state0 初始状态，cov_state0 初始状态的方差
        // ctrl_input 控制输入，即 B 矩阵
        // cov_process 过程噪声方差
        // cov_measure 测量值方差
        KalmanFilter(double state0, double cov_state0, double ctrl_input, double cov_process, double cov_measure);
        // 当有新的测量值时，可以调用此函数，获得一个相对准确的状态值
        double update(double measure_state, double input);

    private:
        // 当前状态
        double state_;
        // 当前状态的方差
        double cov_state_;

        // --- 以下为常量 ---

        // 控制矩阵，一维
        double ctrl_input_;
        // 测量值的方差
        double cov_measure_;
        // 控制量的方差
        double cov_process_;
    };

    // 对上面 KalmanFilter 的封装
    // 适应两变量 (k, b) 的情形
    class CurbEquationFilter
    {
    public:
        // k0 初始斜率
        // b0 初始截距
        CurbEquationFilter(double k0, double b0);
        CurbInfo update(const CurbInfo &measure_val);

    private:
        // 对斜率的滤波器
        KalmanFilter k_filter;
        // 对截距的滤波器
        KalmanFilter b_filter;
    };

    // 针对滑动窗格法优化的最小二乘计算器
    // 当样本中新增或删除一个点的时候，不需要从头开始计算。
    class LeastSquareCalc
    {
    public:
        LeastSquareCalc();
        // 新增
        void push(double x, double y);
        // 去除
        void pop(double x, double y);
        // 清空
        void clear();
        // 计算斜率
        double slope();
        // 计算截距
        double intercept();

    private:
        // xs, ys 均值
        double avg_x_, avg_y_;
        // sigma(xs * xs), sigma(xs * ys)
        double xsxs_, xsys_;
        // 样本数量
        int num_;
    };

    // 路面检测类
    class RoadDetector
    {
    public:
        RoadDetector();
        // 参数
        // frame_data: 一帧中的所有点云
        // 输出：
        // curbs_info: 输出的路沿方程
        // 返回值：路面的点云
        std::vector<Point3d> detect(const std::vector<Point3d> &frame_data, std::vector<CurbInfo> *curbs_info);

    private:
        // 预处理：把点云分成环状
        std::vector<std::vector<hb::Point3d>> preprocess(const std::vector<hb::Point3d> &frame_data);
        // 检测路沿
        std::vector<Point3d> detectCurb(const std::vector<Point3d> &ring_data);
        // 根据左右两边界方程，标记中间夹的所有点为路面
        // left, right: 左右边界方程
        // color: 标记颜色
        std::vector<Point3d> markRoad(const std::vector<Point3d> &frame_data, CurbInfo left, CurbInfo right, int color);

    private:
        // 上次的边界信息
        // 如果某一帧识别失败，就用上次的方程替代
        CurbInfo last_left_;
        CurbInfo last_right_;
        // 左右两边方程的滤波器
        CurbEquationFilter left_filter_;
        CurbEquationFilter right_filter_;
    };

    // 求 z 轴最大高度差
    // [start, end) 半开半闭区间
    inline float max_zdiff(const std::vector<Point3d> &data, size_t start, size_t end, bool increase = true)
    {
        if (!increase)
        {
            size_t tmp = start;
            start = end;
            end = tmp;
        }
        float max = data[start].z;
        float min = data[start].z;
        float z;
        for (size_t i = start; i < end; i++)
        {
            z = data[i].z;
            if (z > max)
                max = z;
            if (z < min)
                min = z;
        }
        return max - min;
    }

    // 根据点云拟合路沿方程
    inline CurbInfo fit_curb(const std::vector<Point3d> &points)
    {
        LeastSquareCalc calc;
        CurbInfo ret;
        // x 轴范围
        ret.x_min = -10;
        ret.x_max = 40;
        for (auto p : points)
            calc.push(p.x, p.y);
        ret.k = calc.slope();
        ret.b = calc.intercept();
        // 如果点数量小于 100, 标记结果不可靠
        ret.available = points.size() > 100;
        return ret;
    }
}
#endif
