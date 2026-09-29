# USER-DEC-SERPENS-DESIGN-0002 — CSAR の撮影（CHILD_NEAR のときは写真確認を延期）（2026-09-29）

Decided by: **User**（Design Agent が記録。原文どおりの要旨。ENTRY-0021 の User への質問への回答）

理由（User）: Floor Watch の優先順位は「認識精度を数秒早く上げること」より「子どもを危険物へ引き寄せないこと」。ボタン電池のような対象は誤飲そのものが重大な危険で（CPSC もボタン電池の誤飲を子どもに重大・致命的な傷害を起こし得る危険として扱っている）、Serpens の照明や視線が誘引になりうる状況では安全側に倒す。「写真を一切撮らない」ではなく、発見した事実を失わないことと子どもを誘導しないことを両立させる。

子どもが `CHILD_NEAR` の場合、Floor Watch の高品質 Photo Verification を延期する:

1. Capture light / line light / 危険物を目立たせる LED は使用しない
2. 危険物を長く見つめたり、頭で指し示したりしない
3. Ambient light だけで、追加の視線誘導や照明なしに撮影できる場合のみ **passive snapshot** を許可
4. それ以外は現在の candidate の位置・confidence・timestamp を Map に保持し、`needs_reinspection=true`
5. CSAR へ移行し、危険物ではなく子ども側を意識した姿勢を取る
6. **blind reverse 300 mm は第一候補にしない**。安全な方向へ turn して forward で離脱する方式を Engineering で検証する
7. 子どもが安全距離外へ移動したら `reinspect_point` を自動生成し、再接近 → 照明 → 高品質撮影 → Risk 再判定
8. `highlight_point` は `CHILD_NEAR` 中の hazard には禁止。Adult-only または child-clear のときのみ使用可
9. CHILD_NEAR の具体的な距離閾値はまだ固定しない。Safety simulation / child model / 実測から Engineering が保守的に導く

優先順位: **child safety > immediate photo verification > notification latency**
