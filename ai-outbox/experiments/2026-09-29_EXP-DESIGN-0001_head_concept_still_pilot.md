# EXP-DESIGN-0001 — 頭部 3 案の静止画予備評価（計画）

- Project: serpens / Agent: Design / Date: 2026-09-29 / 状態: **計画のみ（未実施、HUMAN_EVALUATED = 0）**

## 目的

H0 Egg（現行）/ H1 Bean（推奨）/ H2 Wedge（写実）で、蛇らしさ・かわいさ・怖さの差が大きいかを見る。小さな差を結論にしない。

## 素材

- 3 案を同じ色（sage / ivory）・同じ角度（正面やや上 37°、横、真上）・同じ背景で。H1 は Fusion `Serpens_DESIGN_SD01_BEAN_KNUCKLE_CONCEPT_NOT_FOR_PRINT` から、H0 は FW03、H2 は未モデル（作る必要あり）
- 条件名は見せない。提示順はランダム（既存の `tools/pilot_order.py` を流用できる）

## 尺度（既存の `docs/human_pilot.md` をそのまま使う）

最初に snake_comfort。各画像に snake_likeness / animacy / approachability / affection / fear（1〜7）。任意コメント。
保護者向けに追加 1 問: 「この動き（Child near の姿勢の動画）を見て、子どもと同じ部屋に置いて安心か」（1〜7）

## 仮説

- H1 は H0 より affection・approachability が高く、fear は上がらない
- H2 は snake_likeness が最も高いが fear も上がる
- 蛇が苦手な人（snake_comfort 低）で H1 と H2 の fear の差が大きい

## n と判定

n = 8〜12。差の向きと大きさだけを見る。合格・不合格は決めない。
