# 2026-11-08 M1-T01 LED 缺测/UNKNOWN 不被相对亮度改写

- **状态：** 已确认（M1-T01 主架构师验收 PASS · 2026-11-08）
- **阶段：** M1-T01
- **证据：** `bags/m1_t01_20261108_135953/` · 基线 commit `dbb4e536`（本任务未 commit）
- **相关：** 历史反例 `c2_20261011_s0_unk_B_U_leak_0007` · Hook-1 `bags/vision_train1_e1_demo/`

## 当时目标

修复 `_rollup_leds`：缺测槽位被 `d.get('mean_vmax', d.get('v90', 0.0))` 伪造成 0.0 后，相对亮度规则把 UNKNOWN 改成 off，站点灯汇总误为 FAIL。

## 现象 / 阻塞

历史帧 `c2_20261011_s0_unk_B_U_leak_0007`：`indicator_light_slot_2` 漏检，`reason=yolo_miss_led_slot`，却被写入 `rel_off_override=true` / `state=off`，汇总 FAIL；红灯明细被 off 抢占优先级语义。

## 试过什么（含否决）

- 先写纯逻辑回归（未修源码 6 失败）→ 再最小修复 `_rollup_leds` 及局部 `_slot_brightness` / `_finite_brightness`
- 同环境六帧模型回归：漏检可复现；修复后缺测保持 UNKNOWN；A/B/C 已知 FAIL 未翻 PASS
- **未做：** 改 hook1 生产入口、改 ROI YAML、改权重、commit/push、进入 M1-T02

## 最小修复

- 缺测（字段不存在）≠ 真实 `0.0`
- 缺测/非法值不参与 `row_max`，不写 `rel_to_row_max` / `rel_off_override`
- `UNKNOWN` 即使有测量也不被相对亮度改成 `off`
- `mean_vmax` 优先；仅缺该键时用 `v90`；二者皆缺不伪造

## 测试命令

```bash
# 纯逻辑
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=scripts \
  python3 -m unittest tests.test_roi_decision_rollup -v

# 六帧模型（显式 opt-in）
EVID=bags/m1_t01_20261108_135953
M1_T01_RUN_MODEL=1 M1_T01_EVID_DIR="$PWD/$EVID" \
  YOLO_CONFIG_DIR="$PWD/$EVID/ultralytics_cfg" ULTRALYTICS_OFFLINE=1 \
  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=scripts \
  python3 -m unittest tests.test_m1_t01_hook1_regression.TestM1T01Hook1ModelRegression -v

# 历史保护哈希核对
sha256sum bags/vision_train1_e1_demo/events.jsonl \
  bags/vision_train1_e1_demo/hook1_summary.json \
  bags/vision_5b_c2/train1/weights/best.pt \
  ros2_ws/src/vision_perception/config/roi_rules.yaml
```

## 结论

实施完成，单测与六帧回归通过。

## 主架构师验收（2026-11-08 · PASS）

- 实际 diff 与修改范围符合任务要求。
- 主架构师独立执行 12 项纯逻辑测试及 2 项历史输入/权重检查，通过。
- 使用保存检测框分别重建基线/修复后六帧判定，均匹配快照。
- 18 项受保护文件前后及当前哈希一致。
- 主架构师未重新执行模型推理。
- **不代表**正式独立 Test、90% 或在线巡检通过。

## 下次别踩

缺测测量不得用 `get(..., 0.0)` 填默认值再进相对规则；UNKNOWN 与 off 语义不可混。
