"""ファームウェアを**コンパイルだけ**する（書き込みはしない）。

    .\\.venv\\Scripts\\python.exe tools\\build_firmware.py

目的は構文・型・include・定数・payload の大きさの検証。
`arduino-cli compile` はビルドするだけで、**フラッシュへは一切書き込まない**
（書き込みは `upload` で、このツールは呼ばない）。

検証レベル: コンパイルが通ることは `SOFTWARE_VERIFIED`。
**実機で動くこと（HARDWARE_VERIFIED）とは別**。基板が無いので走らせていない。
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SKETCH = ROOT / "firmware" / "serpens_esp32"
FQBN = "esp32:esp32:XIAO_ESP32S3"       # XIAO ESP32S3（esp32 core 2.0.17 で確認）

# Arduino IDE 2.x が同梱している arduino-cli（この PC で見つかった場所）
BUNDLED = Path(os.environ.get("LOCALAPPDATA", "")) / (
    "Programs/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe")


def find_cli(explicit: str | None) -> Path | None:
    """arduino-cli を探す。PATH → Arduino IDE 同梱 の順。"""
    if explicit:
        return Path(explicit)
    from shutil import which

    found = which("arduino-cli")
    if found:
        return Path(found)
    return BUNDLED if BUNDLED.exists() else None


def main() -> int:
    ap = argparse.ArgumentParser(description="ファームのコンパイル（書き込みはしない）")
    ap.add_argument("--cli", default=None, help="arduino-cli の場所")
    ap.add_argument("--fqbn", default=FQBN)
    ap.add_argument("--sketch", default=None, help="コンパイルするスケッチのフォルダ（既定: firmware/serpens_esp32。頭の XIAO は firmware/serpens_head_xiao）")
    ap.add_argument("--build-path", default=None, help="中間ファイルの置き場所")
    ap.add_argument("--define", action="append", default=[], metavar="NAME[=VAL]",
                    help="コンパイルだけの確認用のマクロ（例: --define SERPENS_SERVO_FAKE=1 --define SERPENS_SERVO_SELFTEST=1。"
                         "実 UART 経路の確認は --define SERVO_TX_PIN=43 --define SERVO_RX_PIN=44）。**書き込み用のファームには使わない**")
    args = ap.parse_args()

    cli = find_cli(args.cli)
    if cli is None or not cli.exists():
        print("arduino-cli が見つかりません。導入手順は firmware/serpens_esp32/README.md。")
        return 2
    cmd = [str(cli), "compile", "--fqbn", args.fqbn]
    if args.build_path:
        cmd += ["--build-path", args.build_path]
    if args.define:
        cmd += ["--build-property", "compiler.cpp.extra_flags=" + " ".join(f"-D{d}" for d in args.define)]
    cmd.append(str(Path(args.sketch).resolve()) if args.sketch else str(SKETCH))
    print("$ " + " ".join(cmd))
    r = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace")
    out = (r.stdout or "") + (r.stderr or "")
    print(out[-4000:])
    if r.returncode != 0:
        print("\n**コンパイル失敗。** 上のエラーを直してから実機の話へ進むこと。")
        return 1
    print("\nコンパイル成功（SOFTWARE_VERIFIED）。**書き込みはしていない**（HARDWARE_UNVERIFIED のまま）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
