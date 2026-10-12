# 2026-10-11 M2 S0-preflight（未冻结 test）

- **状态：** 已确认（preflight Accept；**S0 冻结 Reject**）
- **阶段：** M2 / 5B · S0
- **证据：** `bags/vision_5b_c2/manifests/s0_preflight_*.md|json`（本地）
- **概念：** review_passed≠test≠train；84≠最终 train

## 当时目标

S0-preflight 盘点 + 候选划分；条件满足再冻 test。不训 YOLO、不调 ROI。

## 盘点要点

| 有 RP 的 batch | RP | aug 后代 |
|---|---:|---:|
| b1_d1p2 | 30 | 34 |
| b3_rare | 41 | 40 |
| c22_na_nb | 12 | 0 |
| c22_coff_s8 | 1 | 0 |

跨距批 / aug 批：**0** RP，不可主门禁。

## 方案与否决

- A：b1+na_nb+coff → test RP=43，训 41；缺 UNKNOWN、&lt;60  
- B：b3+na_nb+coff → test RP=54，训 30；同上  
- C：b1+b3 → test 71，训 13；批&lt;3、训过薄、无 UNKNOWN  

**禁止冻结。** 硬缺口：主门禁 RP 无 UNKNOWN；无法在隔离后同时凑规模与可训余量。

## 文档

规范 §8.1 已钉死：漏检/非法输出进分母且计错；UNKNOWN 仅双 UNKNOWN 算对。

## 下一步

独立 raw UNKNOWN（过审）后再开 S0 冻结；本刀无 `test_batches.txt`。
