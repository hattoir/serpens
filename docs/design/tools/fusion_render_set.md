# 人の評価用の描き出し手順（Fusion、Design）

対象: `Serpens_DESIGN_SD01_BEAN_KNUCKLE_RECOVERY_20260929`（元ファイルは使わない）。
Fusion MCP の script（ASCII の文字列だけを使う。日本語を書くと文字化けする: LESSON-DESIGN-0001 #2）で、次を守って描き出す。

1. 表示: `viewport.visualStyle = ShadedVisualStyle`（エッジなし）。REF は非表示、SCENE（硬貨）は表示
2. 姿勢（ジョイント値、このファイルでは J1 は負が頭上げ）:
   - patrol: J1 −5, J2 20, J3 30, J4 −5, J5 −30
   - discovery: 全部 0
   - child_near: J1 −25, J2 −50, J3 −20（Head Yaw ができたら J2 を減らして Head Yaw で子どもを見る）
   - charging: J2〜J5 すべて 45
3. 画角: front = `FrontViewOrientation`（−Y から見た横）、side 代わりの正面 = カメラ eye (−60 cm, 0, 4) → target (0, 0, 4)、
   3/4 = eye (−45, −22, 16) cm → target (−19.5, 0, 4)、up = +Z、top = `TopViewOrientation`。3/4 と正面は isFitView = False で固定
4. 解像度 1200×800、`viewport.saveAsImageFile`
5. 名前: `EVAL_<案>_<姿勢>_<画角>.png`（例 `EVAL_SD01E_child_near_34.png`）。保存先 `docs/design/renders/eval/`
6. 描き出しのあと全ジョイントを 0 に戻す。保存はしない（描き出しは設計を変えない）
