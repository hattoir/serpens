"""フラップ（flank_skirt.py）を付けた状態で、gap_radial の検査を p0..p5 で回す。
使い方: python gap_skirt_run.py <folder> <出力の接頭辞> <joints,カンマ区切り> <pose...>
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import flank_skirt as fs, gap_radial as gr

folder, prefix, joints, poses = Path(sys.argv[1]), sys.argv[2], tuple(sys.argv[3].split(",")), sys.argv[4:]
gr.run(folder, prefix, poses, extra=lambda pose: fs.skirt_meshes(pose, joints=joints))
