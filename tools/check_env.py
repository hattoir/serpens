"""開発環境の健全性チェック。

- opencv-python が入っていないこと（opencv-contrib-python と競合する）
- cv2.aruco が使えること
- 主要パッケージが import できること

使い方: python tools/check_env.py
"""
from __future__ import annotations

import importlib
import importlib.metadata as md
import sys

# import 名 → 表示名
REQUIRED_MODULES: dict[str, str] = {
    "numpy": "numpy",
    "cv2": "opencv-contrib-python",
    "serial": "pyserial",
    "PySide6": "PySide6",
    "yaml": "pyyaml",
    "pytest": "pytest",
    "scservo_sdk": "ftservo-python-sdk",
    "torch": "torch",
    "ultralytics": "ultralytics",
}
# opencv-python 系は opencv-contrib-python と競合。
# feetech-servo-sdk は ftservo-python-sdk と同じ scservo_sdk を上書きしてしまう。
FORBIDDEN_DISTS: tuple[str, ...] = ("opencv-python", "opencv-python-headless", "feetech-servo-sdk")


def main() -> int:
    """チェックを実行し、問題数を終了コードとして返す。"""
    problems = 0
    for dist in FORBIDDEN_DISTS:
        try:
            ver = md.version(dist)
        except md.PackageNotFoundError:
            print(f"[OK]  {dist} は未インストール")
        else:
            print(f"[NG]  {dist} {ver} が入っています → pip uninstall -y {dist}")
            problems += 1

    for mod, label in REQUIRED_MODULES.items():
        try:
            m = importlib.import_module(mod)
        except Exception as e:  # noqa: BLE001 - 何が起きても報告だけする
            print(f"[NG]  {label}: import 失敗 ({e})")
            problems += 1
        else:
            print(f"[OK]  {label} {getattr(m, '__version__', '')}")

    import cv2  # noqa: E402

    try:
        from scservo_sdk import sms_sts  # noqa: F401
    except ImportError:
        print("[NG]  scservo_sdk.sms_sts がありません（ftservo-python-sdk を入れ直す）")
        problems += 1
    else:
        print("[OK]  scservo_sdk.sms_sts（WritePosEx 等）が利用可能")

    if hasattr(cv2, "aruco") and hasattr(cv2.aruco, "ArucoDetector"):
        print("[OK]  cv2.aruco.ArucoDetector が利用可能")
    else:
        print("[NG]  cv2.aruco がありません（opencv-contrib-python が必要）")
        problems += 1

    print("問題なし" if problems == 0 else f"問題 {problems} 件")
    return problems


if __name__ == "__main__":
    sys.exit(main())
