# Handoff — Design（2026-09-30 夜、すき間チェックの続き）

- **Git**: `agent/design-floor-watch`（Design worktree のみ）。main には merge していない
- **Fusion**: 元は未変更。編集は Recovery 複製のみ。追加: `E2-V lower-rear fill`（頭）、`KNUCKLE DRUM J2/J3/J4/J5`（NECK-V と LINK1/2/3 の中）、`FLANK SKIRT STUDY 01`（参照）。**ジョイントの掃引は Fusion では再確認していない**（STL の Python 検査で干渉 0 mm³: p0・p1・p3）

## この区切りでやったこと

- 脇のカバー概念を作って試した → 効かない（増えた）。原因を数字で追い、**玉の脇が痩せて同心のすき間が広がる**ことを特定 → **KNUCKLE DRUM**（円柱に戻す）で胴の層 約 90 % 減
- r の範囲を関節・姿勢ごとに出した（J3〜J5 は r 42.5〜50 の輪）、J1 ピッチの系列（−45〜+22°）
- 頭の下の後ろ: 172 → 2 mm³
- ENTRY-D-0003 の (2)(3)（「形では消せない」）を訂正 → ENTRY-D-0004

## 次に Design がやること

1. **背板の層（z ≥ 77）**: ドームの背板の同心の縁を、玉の脇が痩せても 4 mm 一定になる形にする（背板の A／B の役割を測って確認してから）。約 1020 mm³/姿勢が残っている
2. **J1 ピッチ**: 動作範囲を Engineering と決める。頭の後ろの穴全体を覆う箱形のカバー（頭側の後ろの壁）の設計
3. KNUCKLE DRUM の外観の確認（Fusion の側面・上面の描き出し、人の評価用の画像）と、Fusion での掃引の再確認
4. HEAD E2-V v2 の外観（名前のない体に鋼の外観）の復元、Head Yaw の首の CAD

## Engineering へ / User 判断

- Engineering: ENTRY-D-0004（r の輪と規則の照合、J1 の範囲、背板の扱い、質量 約 60 g 以下の追加）
- User: KNUCKLE DRUM の外観（関節に膝ができる）を進めてよいか
