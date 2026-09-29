# USER-DEC-SERPENS-0003 — 対象年齢・接触の閾値・角度合計の上限・機械式トルクリミッター・継続実行（2026-09-29）

Decided by: **User**（Design Agent が記録。原文の要旨）

1. **OQ-0003 対象年齢**: Primary Target = **3 歳未満**、Safety Design Envelope = **6 歳以下**（ボタン電池等は 6 歳以下まで）。機械設計は Envelope 内の小さい身体寸法側を保守条件にする。**年齢から首径・手首径を推測で決めず、信頼できる Anthropometric Source を探してモデル化する**
2. **OQ-0006 接触の Safety Threshold**: 押付け力・挟み込み力・衝撃力の数値を User は決めない。Engineering が規格・子どもの人体寸法・生体力学の文献・保守的なシミュレーション・H1/H2 以降の実測から暫定値を導く。成人値を子どもへ転用しない。確定まで **PROVISIONAL / SAFETY_UNVERIFIED**。複数候補は安全側を baseline に
3. **OQ-0112 角度合計の上限**: 暫定 Software Safety Limit **145°**（安全が証明された最終値ではない）。145 / 150 / 160 / 170 / 180° を sweep（移動・到達・巻き付きの幾何・首や胸の包囲・挟み込み・配線・機械干渉）。安全と性能の証拠が出た時だけ 145° から上げる提案をしてよい
4. **OQ-0113 機械式トルクリミッター / クラッチ**: 「採用候補」ではなく **設計に含めて検証する Safety Layer**。Slip torque は未確定。候補の下限の領域 約 0.6 N·m、ソフトの目標 約 0.45 N·m で、ソフト＋機械の二重の層として評価。0.5 / 0.6 / 0.7 / 0.8 / 1.0 N·m を sweep（必要なら拡張）: 通常の移動・頭の保持・障害物越え・子どもが引く・指の挟み込み・巻き付き・接触・ギア保護・不要な滑り・故障モード。H1 の実測後に更新
5. **継続自律実行**: これらの Safety 値について逐一 User に質問して止まらない。可逆な範囲で source research → parameterization → simulation → worst-case search → threshold proposal → regression test を繰り返す。**Human Approval が要るのは、実機へ適用する最終 Safety Threshold を緩和する場合だけ**
6. **H0**: 実機の最優先は引き続き H0。実測待ちで止まらず、未測定の摩擦範囲を sweep して Head Yaw / Body Yaw / Wheel Belly の判定の境界を更新し続ける
