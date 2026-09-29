# USER-DEC-SERPENS-0004 — Head Yaw と 145° の関係（2026-09-29）

Decided by: **User**（Design Agent が記録。OPEN-SERPENS-DESIGN-011 への回答）

- 145° は **「Body Curvature Budget」**（胴の中心線がどれだけ曲がって輪を作れるか）と定義する
- **Head Yaw 角は 145° の単純合計には含めない**（頭の向きを変える軸で、同じ角度を単純に足すと幾何の意味が違う）
- ただし Head Yaw は **独立した Safety Axis**。自由には回さない。頭が首・手首・指などに引っ掛かる可能性があるので、別軸として安全評価する
- Engineering は Head Yaw **±15° / ±30° / ±45° / ±60°** を sweep し、CSAR で子どもを見るのに要る角度・Sensor aiming・Sleep pose・首/手首/指への hook geometry・pinch・cable twist・head/body collision・3D swept volume を比べる。**必要な機能を満たす最小の Yaw 範囲**を採用する
- `body curvature <= 145°` だけでは Safety 判定を通さない。**`body curvature + head yaw + actual body dimensions` を使った 3D enclosure / entrapment check を必須**にする
- Head Yaw は「145° に含まれない追加自由度」だが、「Safety budget 外の自由軸」ではない
