# HANDOFF-ENG-VIS-0001 — H2 synthetic vision (in progress, stopped on usage limit)

- Branch `agent/engineering-vision-sim`. Done: VIS-0001 (line-light placement, ENTRY-0015).
- WIP (SOFTWARE only, not yet verified as findings): `serpens/floorwatch/synthetic.py` gains opt-in side walls, nearest-hit ordering,
  floor textures, pile height, blur, ambient drift, auto exposure, line scatter (defaults unchanged; existing 18 floor tests passed
  before the last NaN-guard edit). `simulation/h2_vision.py` = Monte Carlo trial runner (half-res UXGA). No sweep run yet.
- First nominal pass (to verify, may be detector or renderer artefacts): LR44 not flagged metal_disc once sides are drawn
  (bbox elongates → not "roundish"); 6 mm bead / 5 mm magnet ball / 10-yen coin missed; y-offset of candidates for tall objects.
  Debug: bead_6 detects at y≈69–90 when placed at 75 (split into two blobs).
- Next: rerun tests/test_floorwatch.py; one-factor sweeps + MC with ProcessPool; decide whether metal_disc roundness should use
  the top face only; Human Action Queue (`ai-outbox/human-actions/`) not written yet.
