#!/usr/bin/env bash
# [DEPRECATED] 面向已回退的「真脚轮 / caster_mu」试验，不能用于 33a47d7 fixed 球基线。
# 请改用最小场景诊断：
#   ros2 launch simulation_worlds gazebo_robot_v0.launch.py
#   bash scripts/run_creep_min_scene.sh baseline 60
echo "DEPRECATED: fixed-ball baseline → use scripts/run_creep_min_scene.sh" >&2
echo "  bash scripts/run_creep_min_scene.sh baseline 60" >&2
exit 2
