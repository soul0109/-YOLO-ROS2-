# 2026-11-11 B2-fix 正式视觉 Test 评估器修复与重跑 REVIEW

- **状态：** REVIEW
- **仓库：** `/home/charles/inspection-robot` · 修复前 HEAD `66d456e`（本地 ahead 2，origin=`fc411d5`）
- **范围：** 只改评估代码；**未**改模型/ROI/测试集/冻结清单；未启 Gazebo/Nav2/Web
- **证据：** `log/b2_vision_test_20261111/formal_test_run_v2/` · 旧目录标 `INVALID_HARNESS`

## 当时目标

修复 B2 harness 两个已知 bug，并对**同一冻结 test** 离线重跑一次；parity + sanity gate 验收。

## Bug 与修复

1. harness 读 `measurements['panel_state']`/`['leak_state']`，ROI 实际写 `panel`/`leak` → panel/leak 全 UNKNOWN。  
2. 用图像左→右 `slot_states` 对比 GT `states.leds`（设备/model 序）→ 外槽互换。  
3. **修复：** harness 直接调用 Runtime `inspect_station_image`，取 `model_states` / `panel.state` / `leak.state`（含 verdict→state 回退）；保留 `slot_contract`。

## 验收

| 项 | 结果 |
|---|---|
| parity（verify_abc A/B/C） | **PASS**（`tests/test_formal_vision_harness_parity.py`） |
| sanity 拦旧报告 | **PASS**（exit 3；leak correct==0 / panel·leak 全 UNKNOWN / non-UNK acc&lt;0.5） |
| 权重/ROI/freeze/test_batches 哈希 | **不变**（与 B2 冻结一致） |
| 旧报告 | `formal_test_run/INVALID_HARNESS*` 标记，**未删** |

## FORMAL_TEST_v2（一次）

| 指标 | 无效旧报告 | **v2（有效）** |
|---|---:|---:|
| per-object | 0.346（作废） | **0.849**（297/350） |
| station rollup | 0.30（作废） | **0.857**（60/70） |
| UNKNOWN GT | 1.0（作废） | **0.619**（13/21） |
| leak | 0.0 | **1.0** |
| panel | ~0.21 | **0.886** |

仍 **&lt;90%**。不重训、不采新 Test、不回测调参。下一步选项：另立候选重训 + **新**独立 Test，或先做 B3/作品集。

## 哈希（节选）

见 `log/b2_vision_test_20261111/07_artifact_sha256.txt` 与 `formal_test_run_v2/07_artifact_sha256.txt`。  
weight `a1d2507a…` · ROI `1e1f1fd2…` · freeze `31d83e42…` · test_batches `3292659f…`。

## 下次别踩

正式评估必须走 Runtime 同路径；sanity gate 拦住「一类全 UNKNOWN / correct==0」类 harness 事故。
