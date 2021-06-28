#include "road_detector.h"

namespace hb
{
    RoadDetector::RoadDetector() : left_filter_(0, 10), right_filter_(0, -10) {}

    std::vector<Point3d> RoadDetector::detect(const std::vector<Point3d> &frame_data, std::vector<CurbInfo> *curbs_info)
    {
        // 第一步：预处理
        std::vector<std::vector<hb::Point3d>> rings = preprocess(frame_data);

        // 第二步：检测边界点
        // 所有的边界点
        std::vector<Point3d> all_curbs;
        for (size_t i = 0; i < rings.size(); i++)
        {
            auto curbs = detectCurb(rings[i]);
            all_curbs.reserve(all_curbs.size() + curbs.size());
            all_curbs.insert(all_curbs.end(), curbs.begin(), curbs.end());
        }
        // printf("Find %lu / %lu point of curbs\n", all_curbs.size(), frame_data.size());

        // 分离左右边界，根据 Y 轴正负
        std::vector<Point3d> left_curbs;
        std::vector<Point3d> right_curbs;
        for (auto p : all_curbs)
        {
            if (p.y >= 0)
            {
                left_curbs.push_back(p);
            }
            else
            {
                right_curbs.push_back(p);
            }
        }
        // printf("Left: %lu, Right: %lu\n", left_curbs.size(), right_curbs.size());

        // 第三步：拟合
        // 最小二乘拟合
        CurbInfo left, right;

        left = fit_curb(left_curbs);
        right = fit_curb(right_curbs);

        if (left.available)
            last_left_ = left;
        if (right.available)
            last_right_ = right;

        // 第四步：卡尔曼滤波
        left = left_filter_.update(left);
        right = right_filter_.update(right);

        // 至此，已经有左右两边界的方程了
        // 输出到 curbs_info
        if (curbs_info)
        {
            curbs_info->clear();
            curbs_info->push_back(left);
            curbs_info->push_back(right);
        }
        // 最后一步：标记两边界之间的点为路面
        std::vector<Point3d> roads = markRoad(frame_data, left, right, 0x00ff00);
        return roads;
    }

    // 预处理
    std::vector<std::vector<hb::Point3d>> RoadDetector::preprocess(const std::vector<hb::Point3d> &frame_data)
    {
        const hb::Point3d &p0 = frame_data[0];
        const hb::Point3d &p1 = frame_data[1];
        float theta0 = std::atan2(p0.y, p0.x);
        float theta1 = std::atan2(p1.y, p1.x);

        if (theta1 < theta0)
        {
            // 不处理顺时针扫描的点
            std::cerr << "Error: Bad frame_data";
        }

        int sign0 = theta0 >= 0 ? 1 : -1;

        std::vector<std::vector<hb::Point3d>> out;
        size_t len = frame_data.size();
        size_t start = 0;
        float th;
        int sign;
        for (size_t i = 1; i < len;)
        {
            th = std::atan2(frame_data[i].y, frame_data[i].x);
            while (th > theta0)
            {
                i++;
                // 用 break 会导致极端情况下数组越界
                if (i >= len)
                    goto slice;
                th = std::atan2(frame_data[i].y, frame_data[i].x);
            }

            sign = th >= 0 ? 1 : -1;
            // th 和 theta0 异号
            while (sign + sign0 == 0)
            {
                i++;
                if (i >= len)
                    goto slice;
                th = std::atan2(frame_data[i].y, frame_data[i].x);
                sign = th > 0 ? 1 : -1;
            }

            while (th < theta0)
            {
                i++;
                if (i >= len)
                    goto slice;
                th = std::atan2(frame_data[i].y, frame_data[i].x);
            }

        // 对 frame_data 切片
        slice:
            // 把 frame_data 范围 [start, i) 的点分离出来
            // 称为一个 ring
            if (start != i)
            {
                std::vector<hb::Point3d> ring;
                for (size_t j = start; j < i; j++)
                    ring.push_back(frame_data[j]);
                out.push_back(ring);
                start = i;
            }
            else
            {
                if (i != len - 1)
                    std::cerr << "Error: Unexpected i: " << i << "\n";
                break;
            }
        }
        return out;
    }

    // 检测路沿点云
    std::vector<Point3d> RoadDetector::detectCurb(const std::vector<Point3d> &ring_data)
    {
        std::vector<Point3d> out;
        size_t len = ring_data.size();

        // 高度差阈值、斜率阈值
        float zthrehold = 0.06;
        float kthrehold = 0.7;
        // 窗格大小
        size_t window_size = 12;

        hb::LeastSquareCalc calc;

        // 是否匹配成功
        // ok[0] z轴高度差
        // ok[1] 拟合直线斜率
        bool ok[] = {false, false};

        // 从中心向左移动的窗格
        for (auto it = ring_data.begin(); it != ring_data.end(); ++it)
        {
            // 只处理一、二向限的点
            if ((it + window_size - 1)->y < 0)
                break;

            //---------
            // 检测高度差
            //---------
            if (!ok[0])
            {
                size_t start = it - ring_data.begin();
                size_t end = it + window_size - ring_data.begin();

                float zdiff = max_zdiff(ring_data, start, end);
                if (std::abs(zdiff) > zthrehold)
                {
                    out.insert(out.end(), it, it + window_size);
                    ok[0] = true;
                }
            }
            //---------

            //---------
            // 检测斜率
            //---------
            if (!ok[1])
            {
                if (it == ring_data.begin())
                {
                    for (int i = 0; i < window_size; i++)
                    {
                        auto p = *(it + i);
                        calc.push(p.y, p.x);
                    }
                }
                else
                {
                    auto p = *(it - 1);
                    calc.pop(p.y, p.x);
                    p = *(it + window_size - 1);
                    calc.push(p.y, p.x);
                }
                double k = 1 / calc.slope();

                if (std::abs(k) < kthrehold)
                {
                    out.insert(out.end(), it, it + window_size);
                    ok[1] = true;
                }
            }
            //---------

            if (ok[0] && ok[1])
                break;
        }

        // 清除状态
        calc.clear();
        ok[0] = false;
        ok[1] = false;
        // 从中心向右移动的窗格
        for (auto it = ring_data.rbegin(); it != ring_data.rend(); ++it)
        {
            // 只处理三、四向限的点
            if ((it + window_size - 1)->y > 0)
                break;

            //---------
            // 检测高度差
            //---------
            if (!ok[0])
            {
                size_t start = (it + window_size).base() - ring_data.begin();
                size_t end = it.base() - ring_data.begin();

                float zdiff = max_zdiff(ring_data, start, end);
                if (std::abs(zdiff) > zthrehold)
                {
                    out.insert(out.end(), (it + window_size - 1).base(), it.base());
                    ok[0] = true;
                }
            }
            //---------

            //---------
            // 检测斜率
            //---------
            if (!ok[1])
            {
                if (it == ring_data.rbegin())
                {
                    for (int i = 0; i < window_size; i++)
                    {
                        auto p = *(it + i);
                        calc.push(p.y, p.x);
                    }
                }
                else
                {
                    auto p = *(it - 1);
                    calc.pop(p.y, p.x);
                    p = *(it + window_size - 1);
                    calc.push(p.y, p.x);
                }
                double k = 1 / calc.slope();
                if (std::abs(k) < kthrehold)
                {
                    out.insert(out.end(), (it + window_size - 1).base(), it.base());
                    ok[1] = true;
                }
            }

            //---------
            if (ok[0] && ok[1])
                break;
        }
        return out;
    }

    std::vector<Point3d> RoadDetector::markRoad(const std::vector<Point3d> &frame_data, CurbInfo left, CurbInfo right, int color)
    {
        std::vector<Point3d> ret;
        if (!left.available)
            left = last_left_;
        if (!right.available)
            right = last_right_;

        float y_max, y_min;
        for (auto p : frame_data)
        {
            // y = k * x + b
            y_max = left.k * p.x + left.b;
            y_min = right.k * p.x + right.b;
            if (p.y < y_max && p.y > y_min)
                p.rgb = color;
            ret.push_back(p);
        }
        return ret;
    }

    //---------------------------------
    // LeastSquareCalc 相关函数实现
    //---------------------------------
    LeastSquareCalc::LeastSquareCalc()
    {
        clear();
    }
    void LeastSquareCalc::push(double x, double y)
    {
        avg_x_ = (avg_x_ * num_ + x) / (num_ + 1);
        avg_y_ = (avg_y_ * num_ + y) / (num_ + 1);
        xsxs_ = xsxs_ + x * x;
        xsys_ = xsys_ + x * y;
        num_ += 1;
    }
    void LeastSquareCalc::pop(double x, double y)
    {
        avg_x_ = (avg_x_ * num_ - x) / (num_ - 1);
        avg_y_ = (avg_y_ * num_ - y) / (num_ - 1);
        xsxs_ = xsxs_ - x * x;
        xsys_ = xsys_ - x * y;
        num_ = num_ - 1;
    }
    void LeastSquareCalc::clear()
    {
        avg_x_ = 0;
        avg_y_ = 0;
        xsxs_ = 0;
        xsys_ = 0;
        num_ = 0;
    }
    double LeastSquareCalc::slope()
    {
        if (num_ == 0)
            return 0;
        double numerator = xsys_ - num_ * avg_x_ * avg_y_;
        double denominator = xsxs_ - num_ * avg_x_ * avg_x_;
        return numerator / denominator;
    }
    double LeastSquareCalc::intercept()
    {
        if (num_ == 0)
            return 0;
        return avg_y_ - slope() * avg_x_;
    }

    //---------------------------------
    // KalmanFilter 相关函数实现
    //---------------------------------
    KalmanFilter::KalmanFilter(double state0, double cov_state0, double ctrl_input, double cov_process, double cov_measure)
    {
        state_ = state0;
        cov_state_ = cov_state0;
        ctrl_input_ = ctrl_input;
        cov_process_ = cov_process;
        cov_measure_ = cov_measure;
    }

    // 当有一个新的测量值时，可以调用此函数，获得一个相对准确的状态值
    double KalmanFilter::update(double measure_state, double input)
    {
        // 预估值
        double predict_state = state_ + ctrl_input_ * input;
        // 预估值的方差
        double predict_cov_state = cov_state_ + cov_process_;
        // 卡尔曼增益
        double kalman_gain = predict_cov_state / (predict_cov_state + cov_measure_);
        // 对预估值和测量值加权
        double updated_state = (1 - kalman_gain) * predict_state + kalman_gain * measure_state;
        double updated_cov_state = (1 - kalman_gain) * predict_cov_state;
        // 更新
        this->state_ = updated_state;
        this->cov_state_ = updated_cov_state;
        return updated_state;
    }

    //---------------------------------
    // CurbEquationFilter 相关函数实现
    //---------------------------------
    CurbEquationFilter::CurbEquationFilter(double k0, double b0) : k_filter(k0, 1, 0, 0.25, 1.96),
                                                                   b_filter(b0, 1, 0, 0.35, 2.25)
    {
    }
    CurbInfo CurbEquationFilter::update(const CurbInfo &measure_val)
    {
        CurbInfo ret = measure_val;
        if (!ret.available)
            return ret;
        ret.k = k_filter.update(measure_val.k, 0);
        ret.b = b_filter.update(measure_val.b, 0);
        return ret;
    }

}