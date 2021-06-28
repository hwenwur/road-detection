#include <iostream>
#include <chrono>
#include <fstream>
#include <vector>
#include <cstdlib>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <visualization_msgs/Marker.h>

#include "road_detector.h"

// 读取点云数据
// frame_idx: 点云帧的文件名, 范围：[1, 108]
std::vector<hb::Point3d> load_frame(const std::string &base_path, int frame_idx)
{
    std::string filename = std::to_string(frame_idx) + ".txt";
    std::string path = base_path + "/" + filename;
    std::ifstream fin(path);
    if (!fin.good())
    {
        std::cerr << "Failed to read file: " << path << "\nExiting...\n";
        std::exit(-1);
    }
    std::vector<hb::Point3d> out;
    hb::Point3d p;
    while (fin.good())
    {
        fin >> p.x >> p.y >> p.z;
        p.intensity = 4096;
        p.rgb = 0xffffff;
        out.push_back(p);
    }
    return out;
}

sensor_msgs::PointField create_pointfield(const std::string &name, uint32_t offset)
{
    sensor_msgs::PointField field;
    field.name = name;
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.offset = offset;
    field.count = 1;
    return field;
}

void publish_pointcloud(ros::Publisher &pub, const std::vector<hb::Point3d> &points)
{
    static std::vector<sensor_msgs::PointField> fields = {create_pointfield("x", 0),
                                                          create_pointfield("y", 4),
                                                          create_pointfield("z", 8),
                                                          create_pointfield("i", 12),
                                                          create_pointfield("rgb", 16)};

    const uint32_t point_step = sizeof(hb::Point3d);

    std::vector<uint8_t> data;
    data.reserve(points.size() * point_step);
    unsigned char u[sizeof(hb::Point3d)];
    for (size_t i = 0, len = points.size(); i < len; i++)
    {
        const hb::Point3d *p = &points[i];
        std::memcpy(&u[0], p, point_step);
        for (uint32_t j = 0; j < point_step; j++)
            data.push_back(u[j]);
    }

    sensor_msgs::PointCloud2 pointcloud;
    pointcloud.header.frame_id = "velo_link";
    pointcloud.header.stamp = ros::Time::now();
    pointcloud.fields = fields;
    pointcloud.height = 1;
    pointcloud.width = points.size();
    pointcloud.is_bigendian = false;
    pointcloud.point_step = point_step;
    pointcloud.row_step = points.size() * point_step;
    pointcloud.data = data;
    pointcloud.is_dense = true;
    pub.publish(pointcloud);
}

// 在 rviz 中，标记一条线段
// id 相同，表示修改原有线段位置
void publish_line_list(const ros::Publisher &pub, const std::vector<hb::Point3d> &points, int id)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "velo_link";
    marker.header.stamp = ros::Time::now();

    marker.ns = "curb";
    marker.id = id;

    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1;

    // 线宽
    marker.scale.x = 0.1;

    // 颜色，RGBA
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 1;

    // 保留时间
    marker.lifetime = ros::Duration(0);

    marker.frame_locked = true;

    for (auto p : points)
    {
        geometry_msgs::Point pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = p.z;
        marker.points.push_back(pt);
    }
    pub.publish(marker);
}

void publish_curbs(const ros::Publisher &pub, const std::vector<hb::CurbInfo> &curbs_info)
{
    float z = -1.7;
    int id = 0;
    hb::Point3d p;
    for (auto curb : curbs_info)
    {
        if (!curb.available)
            continue;
        std::vector<hb::Point3d> pts;
        // 线段起点
        p.x = curb.x_min;
        p.y = p.x * curb.k + curb.b;
        p.z = z;
        pts.push_back(p);

        // 线段终点
        p.x = curb.x_max;
        p.y = p.x * curb.k + curb.b;
        p.z = z;
        pts.push_back(p);
        publish_line_list(pub, pts, id);

        id++;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "roaddetection_node");

    using std::chrono::duration_cast;
    using std::chrono::high_resolution_clock;
    using std::chrono::milliseconds;

    std::string pc_topic = "color_cloud";
    std::string curb_topic = "curb_line";

    ros::NodeHandle nh;
    ros::Publisher pc_pub = nh.advertise<sensor_msgs::PointCloud2>(pc_topic, 10);
    ros::Publisher curb_pub = nh.advertise<visualization_msgs::Marker>(curb_topic, 10);

    std::string data_dir;
    if (!ros::param::get("~data_dir", data_dir))
    {
        std::cerr << "Warn: can't access parameter: data_dir\n";
        std::exit(-1);
    }

    std::vector<std::vector<hb::Point3d>> all_frames;
    for (int i = 0; i < 108; i++)
    {
        all_frames.push_back(load_frame(data_dir, i + 1));
    }

    int frame_idx = 1;

    std::vector<hb::CurbInfo> curbs_info;
    hb::RoadDetector detector;

    // 频率：10hz
    ros::Rate rate(10);

    std::cerr << "Start processing...\n";
    while (nh.ok())
    {
        auto frame = all_frames[frame_idx - 1];

        auto t1 = high_resolution_clock::now();
        auto color_point = detector.detect(frame, &curbs_info);
        auto t2 = high_resolution_clock::now();

        // 计算处理耗时，单位: ms
        long cost_time = duration_cast<milliseconds>(t2 - t1).count();

        publish_pointcloud(pc_pub, color_point);
        publish_curbs(curb_pub, curbs_info);

        std::cerr << "Process frame[" << frame_idx << "] cost " << cost_time << "ms\n";

        frame_idx++;
        if (frame_idx > 108)
            frame_idx = 1;

        rate.sleep();
    }
    return 0;
}
