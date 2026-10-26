// D455 双目回放节点: infra1/infra2 时间同步后喂 ORB_SLAM3 TrackStereo
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/core.hpp>
#include <atomic>
#include <thread>
#include <chrono>
#include <unistd.h>
#include "System.h"

class Grabber {
public:
    Grabber(ORB_SLAM3::System* p) : mpSLAM(p) {}
    void Grab(const sensor_msgs::ImageConstPtr& L, const sensor_msgs::ImageConstPtr& R) {
        cv::Mat imL, imR;
        try {
            imL = cv_bridge::toCvShare(L, "mono8")->image;
            imR = cv_bridge::toCvShare(R, "mono8")->image;
        } catch (cv_bridge::Exception& e) { ROS_ERROR("cv_bridge: %s", e.what()); return; }
        mpSLAM->TrackStereo(imL, imR, L->header.stamp.toSec());
        long n = ++nSync;
        if (n % 30 == 0) ROS_INFO("已处理 %ld 个同步帧对", n);
    }
    ORB_SLAM3::System* mpSLAM;
    std::atomic<long> nSync{0};
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "d455_stereo_slam");
    if (argc < 3) { std::cout << "用法: " << argv[0] << " 词库 配置.yaml [viewer 0/1]\n"; return 1; }
    bool viewer = (argc >= 4 && std::string(argv[3]) == "1");

    ros::NodeHandle nh, pnh("~");
    std::string lt, rt;
    pnh.param<std::string>("left_topic",  lt, "/camera/infra1/image_rect_raw");
    pnh.param<std::string>("right_topic", rt, "/camera/infra2/image_rect_raw");

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, viewer);
    ROS_INFO("SLAM 就绪, 等待图像 [%s] + [%s]", lt.c_str(), rt.c_str());

    Grabber gb(&SLAM);
    std::atomic<long> nL{0}, nR{0};
    ros::Subscriber cL = nh.subscribe<sensor_msgs::Image>(lt, 1, [&](const sensor_msgs::Image::ConstPtr&){ nL++; });
    ros::Subscriber cR = nh.subscribe<sensor_msgs::Image>(rt, 1, [&](const sensor_msgs::Image::ConstPtr&){ nR++; });

    message_filters::Subscriber<sensor_msgs::Image> subL(nh, lt, 1);
    message_filters::Subscriber<sensor_msgs::Image> subR(nh, rt, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPol;
    message_filters::Synchronizer<SyncPol> sync(SyncPol(10), subL, subR);
    sync.registerCallback(boost::bind(&Grabber::Grab, &gb, _1, _2));

    ros::Timer diag = nh.createTimer(ros::Duration(5.0), [&](const ros::TimerEvent&){
        if (gb.nSync == 0)
            ROS_WARN("5秒无同步帧对: L=%ld R=%ld -- 查话题名(rosbag info)或bag是否在播", (long)nL, (long)nR);
    });

    ros::spin();
    // 后台线程保存 + 看门狗：保存卡死也能退出
    std::thread([](ORB_SLAM3::System* S){
        S->Shutdown();
        S->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
        S->SaveTrajectoryTUM("CameraTrajectory.txt");
        ROS_INFO("轨迹已保存");
        _exit(0);
    }, &SLAM).detach();
    for (int i = 0; i < 25; i++) std::this_thread::sleep_for(std::chrono::seconds(1));
    ROS_WARN("保存超时25s, 强制退出");
    _exit(0);
}
