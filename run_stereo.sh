#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash
export LD_LIBRARY_PATH=$HOME/ORB_SLAM3/lib:$HOME/ORB_SLAM3/Thirdparty/g2o/lib:$HOME/ORB_SLAM3/Thirdparty/DBoW2/lib:${LD_LIBRARY_PATH:-}
for d in $HOME/ORB_SLAM3/Thirdparty/Sophus/lib $HOME/ORB_SLAM3/Thirdparty/Pangolin/build/src; do
  [ -d "$d" ] && export LD_LIBRARY_PATH=$d:$LD_LIBRARY_PATH
done

BAG="${2:-$HOME/bags/d435i_test.bag}"
NODE=$HOME/slam_test/build/d455_stereo_slam
VOCAB=$HOME/ORB_SLAM3/Vocabulary/ORBvoc.txt
YAML=$HOME/slam_test/D455.yaml
for f in "$BAG" "$NODE" "$VOCAB" "$YAML"; do
  [ -f "$f" ] || { echo "缺少 $f，先完成前面的步骤"; exit 1; }
done

cd $HOME/slam_test
 $NODE $VOCAB $YAML ${1:-0} > slam_run.log 2>&1 &
NODE_PID=$!
echo "节点启动中(pid $NODE_PID)，词库加载约20~40秒，另开窗口: tail -f slam_run.log"
sleep 30
echo ">>> 开始播放 bag（空格=暂停/继续）"
rosbag play -r 0.1 $BAG
echo ">>> 播完，按一次 Ctrl+C 结束节点并自动保存轨迹"
wait $NODE_PID || true
ls -lh CameraTrajectory.txt KeyFrameTrajectory.txt 2>/dev/null || echo "没找到轨迹文件，看 slam_run.log"
