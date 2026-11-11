# 2026-11-11 B2 正式视觉 Test 冻结与执行 REVIEW

> **评估器无效，已被本条目取代：** 见 [2026-11-11-B2正式视觉Test评估器修复与重跑REVIEW](2026-11-11-B2正式视觉Test评估器修复与重跑REVIEW.md)。  
> 本条 `FORMAL_TEST` ~34.6% 因 harness bug（panel/leak 键名错误 + LED 图像槽/设备序混淆）判 **INVALID_HARNESS**；冻结 test/权重/ROI **仍有效**，不删原文。

- **状态：** REVIEW（评估报告作废；采集/冻结事实保留）
- **仓库：** `/home/charles/inspection-robot` · 批前 HEAD `fc411d5` = origin · tag `mvp-r7-console-20261111`
- **证据：** `log/b2_vision_test_20261111/` · `bags/vision_5b_test_20261111/`（gitignore，磁盘保留）

## 当时目标

按 M2-5B 合同冻结候选权重、独立原始 Test 采集、hash-gated 一次性评估；不重训、不改冻结导航/几何、不宣称 90%。

## 完成项

1. **文档：** 主手册 §1/§4、NOW、滚动计划同步为 B2；保留 10/10、2/10 历史事实。
2. **stop：** `scripts/run_mission_full.sh stop` 按本仓库 launch 树 + `test_room` gzserver 精确清理，输出 `STACK_CLEAR`（见 `01_*`）。全栈再冷启曾遇 `amcl_missing` 时序失败（未改 AMCL；本批改走 Gazebo-only 采集）。
3. **候选：** `bags/vision_5b_c2/train1/weights/best.pt` → `models/weights/inspection_train1_20261012.pt`（sha256 `a1d2507a…fed4`）+ args/dev report/split + `models/candidates/inspection_train1_20261012.candidate.json`（含 ROI hash / conf=0.25 / code HEAD）。**未重训。**
4. **采集：** Gazebo-only（无 Nav2）；3 独立 batch；种子 ≥401；`synthetic=false`；目录 `bags/vision_5b_test_20261111/`。配额：主距 60（A/B/C 各 20）+ 1.0×5 + 1.5×5；UNKNOWN 主距 21；难例 12。
5. **冻结：** `splits/test_batches.txt` + `manifests/test_freeze_20261111.json`（sha256 `31d83e42…`）；候选 `formal_test.frozen=true`。
6. **Harness：** `scripts/formal_vision_test_run.py`；dev dry-run 标 `DRY_RUN_NOT_TEST`；正式跑拒绝未冻结/hash 不符。

## 正式 Test 结果（一次）

| 指标 | 值 |
|---|---|
| report_tag | `FORMAL_TEST` |
| frames | 70 |
| per-object agreement | **0.346**（121/350） |
| station rollup | **0.30**（21/70） |
| UNKNOWN GT | **1.0**（21/21） |
| leak / panel / led 分项 | leak **0.0**；panel ~0.21；led 槽位不均 |

**结论：未过 ≥90% 门禁。不得用本冻结集回测调参。** 建议：另立候选（可重训检测器 / 复核 ROI 与漏液框关联）+ **新**独立 Test。

## 试过什么（含否决）

- 否决：用 C1/C2 / aug / soft val 充正式 Test。
- 否决：因未达门禁而改 ROI/权重后再测同一 test。
- 全栈起停验证：stop 清净；start 遇 AMCL 未出 pose（已知环境时序），不在本批修导航。

## 下次别踩

正式 Test 冻结后只允许报告；失败开新集，不在旧 test 上刷分。
