orbslam3 推理rosbag获取轨迹文件

cd ~/slam_test && mkdir -p build && cd build && cmake .. && make -j2
~/slam_test/run_stereo.sh 0 ~/bags/d455_test1.bag
tail -f ~/slam_test/slam_run.log