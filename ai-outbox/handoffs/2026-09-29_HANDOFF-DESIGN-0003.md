# Handoff — Design（2026-09-29 夜、Continuous Design Mode）

- **Git**: `agent/design-floor-watch`（Design worktree のみ、push 済み）。main には merge していない
- **Fusion**: 元 `Serpens_DESIGN_SD01_BEAN_KNUCKLE_CONCEPT_NOT_FOR_PRINT` は v6 のまま。編集は Recovery copy `Serpens_DESIGN_SD01_BEAN_KNUCKLE_RECOVERY_20260929` だけ（v10 以降）。すべて別コンポーネントで追加し、元の部品は変えていない

## 現在の Design の第一候補（CAD_CONCEPT、HUMAN_EVALUATED = 0）

頭 **SD-01E3**（= E2 の顔 ＋ 横の口の線）＋ J1 は **VISOR**（首の芯＋頭巾＋襟）＋ 胴は同心ナックル（背板 r45 flush）＋ 顎板のそり（CHIN SHIELD）＋ 照明は頬の下（斜め）と口の線（通常）

## この区切りでやったこと

- Engineering の回答 ENTRY-0004 / 0015 / 0019 / 0020 / 0021 に **ENTRY-0022** で回答（ライン光は眉で確定、照明の位置、スキッドの形、後ろ向きの退避の懸念、ENTRY 番号の重複）
- 照明の計算 `docs/design/tools/light_placement.py`: 斜め 40 → 15 倍、通常 33 → 11 倍
- Fusion: LIGHT+SKID、CHIN SHIELD の重ねコンポーネント、描き出し
- 6 本目の見た目の比較（`assets/sixth_motor_silhouettes.png`）: Head Yaw の利点は CSAR の視線だけになった
- ENTRY-0011 の案 A を部品ごとに当てた（**ENTRY-0023**）: 電池だけ 12 mm はみ出す

## Engineering へ

ENTRY-0022（照明 2 灯ずつで検出が成り立つか・後ろ向きの退避・番号の付け方）、ENTRY-0023（電池は尾 +12 mm か 93 mm 以下か）、ENTRY-0013（VISOR の判定）

## User 判断

- ENTRY-0021（Engineering）: 子どもが近い時は撮影を後回しにする（確認が遅れる向き）を受け入れるか
- 6 本目（H0 の結果しだい、User Decision 2 のまま）

## 次に試す案

顎板の厚み・縁（3 mm・R1）、VISOR の芯の色、Head Yaw の上の首（幅 52〜56）の CAD、人の評価の画像の最終セット（E3 ＋ VISOR ＋ 顎板で撮り直し）
