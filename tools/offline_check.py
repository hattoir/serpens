"""ネットワークを完全に遮断した状態で、アプリが最後まで動くか確かめる。

**展示会場にネットは無い前提**です。ultralytics は重みの自動ダウンロードや更新確認で外へ出ようとするため、
ここで「外へ出たら即エラーになる」状態にして起動し、最後まで走り切ることを確認します。

使い方:
  python tools/offline_check.py                       # --sim --no-gui
  python tools/offline_check.py --camera 0            # 実カメラ + YOLO も含めて確認
  python tools/offline_check.py --camera 0 --gui      # GUI も出して確認
"""
from __future__ import annotations

import argparse
import socket
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

BLOCKED = "ネットワークは遮断されています（オフライン検証中）"
ALLOW_LOCAL = ("127.0.0.1", "::1", "localhost")


def block_network() -> None:
    """ローカル以外への通信を禁止する。GUI（Qt）はローカルのソケットを使うことがあるので許可する。"""
    real_socket = socket.socket

    class Guarded(real_socket):  # type: ignore[misc, valid-type]
        def connect(self, address):  # type: ignore[no-untyped-def]
            host = address[0] if isinstance(address, tuple) else str(address)
            if host not in ALLOW_LOCAL:
                raise OSError(f"{BLOCKED}: connect({host})")
            return super().connect(address)

    def no_dns(*a: object, **k: object) -> None:
        raise OSError(f"{BLOCKED}: getaddrinfo")

    socket.socket = Guarded          # type: ignore[assignment]
    socket.getaddrinfo = no_dns      # type: ignore[assignment]
    socket.create_connection = no_dns  # type: ignore[assignment]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", default=None, help="カメラ番号 / 動画 / 画像")
    ap.add_argument("--gui", action="store_true", help="GUI も出す")
    ap.add_argument("--seconds", type=float, default=8.0)
    args = ap.parse_args()
    block_network()
    from serpens.app import main as app_main

    argv = ["--sim", "--seconds", str(args.seconds)]
    if args.camera is not None:
        argv += ["--camera", args.camera]
    if not args.gui:
        argv.append("--no-gui")
    print(f"[オフライン検証] python -m serpens.app {' '.join(argv)}")
    code = app_main(argv)
    print("[オフライン検証] 最後まで動きました" if code == 0 else f"[オフライン検証] 失敗（終了コード {code}）")
    return code


if __name__ == "__main__":
    sys.exit(main())
