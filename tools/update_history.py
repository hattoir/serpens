"""`agent/HISTORY.md` の**自動部分**を作り直す（手書きの「軌跡」の部分は残す）。

    python tools/update_history.py            # agent/HISTORY.md の <!-- BEGIN:auto --> 〜 <!-- END:auto --> を更新
    python tools/update_history.py --check   # 自動部分が最新かだけを調べる（古ければ終了コード 1）

自動部分の中身（すべて git と、共有ファイルの実物から機械的に作る。**推測で書かない**）:
  1. ブランチの地図（先頭のコミット・日付・main との差のコミット数）
  2. 日付ごとの全コミット（どのブランチにあるか。件名）
  3. 判断の記録（各ブランチの `agent/DECISIONS.md` の見出し。重複は 1 つにまとめる）
  4. Design ↔ Engineering の ENTRY の索引（`ai-shared/integration-log.md` があれば。`From` / `Area`）
手書きの部分（`<!-- BEGIN:narrative -->` 〜 `<!-- END:narrative -->`）は**この道具では触らない**。作業のまとまりごとに、人（Agent）が追記する（`CLAUDE.md`「軌跡の記録」）。
"""
from __future__ import annotations

import argparse
import re
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "agent" / "HISTORY.md"
BEGIN, END = "<!-- BEGIN:auto -->", "<!-- END:auto -->"


def git(*args: str, check: bool = True) -> str:
    r = subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True, encoding="utf-8")
    if check and r.returncode != 0:
        raise RuntimeError(f"git {' '.join(args)}: {r.stderr.strip()}")
    return r.stdout


def branches() -> list[str]:
    out = git("for-each-ref", "--format=%(refname:short)", "refs/heads")
    return [b for b in out.split() if b]


def ai_shared() -> Path | None:
    """ai-shared/ は git 管理外（main の作業ディレクトリ）。worktree からは共通の .git の親を探す。"""
    common = Path(git("rev-parse", "--git-common-dir").strip())
    common = common if common.is_absolute() else (ROOT / common)
    p = common.resolve().parent / "ai-shared"
    return p if p.exists() else None


def build() -> str:
    bs = branches()
    members: dict[str, set[str]] = {b: set(git("rev-list", b).split()) for b in bs}
    main = members.get("main", set())
    lines: list[str] = []
    A = lines.append

    # 1. ブランチの地図
    A("### 1. ブランチの地図\n")
    A("| ブランチ | 先頭 | 日付 | main との差（コミット数）| 先頭の件名 |\n|---|---|---|---|---|")
    for b in sorted(bs):
        head = git("log", "-1", "--format=%h|%ad|%s", "--date=short", b).strip().split("|", 2)
        ahead = len(members[b] - main)
        A(f"| `{b}` | {head[0]} | {head[1]} | {ahead} | {head[2][:90]} |")
    A("")

    # 2. 日付ごとの全コミット
    A("### 2. 日付ごとの全コミット（古い順。ブランチ = そのコミットを含むブランチ。main に含まれるものは main）\n")
    rows = git("log", "--all", "--reverse", "--format=%H|%h|%ad|%s", "--date=short").strip().splitlines()
    by_day: dict[str, list[tuple[str, str, str]]] = defaultdict(list)
    seen: set[str] = set()
    for row in rows:
        full, short, day, subj = row.split("|", 3)
        if full in seen:
            continue
        seen.add(full)
        where = ["main"] if full in main else sorted(b for b in bs if full in members[b])
        by_day[day].append((short, ", ".join(where), subj))
    for day in sorted(by_day):
        A(f"#### {day}（{len(by_day[day])} コミット）\n")
        for short, where, subj in by_day[day]:
            A(f"- `{short}` [{where}] {subj}")
        A("")

    # 3. 判断の記録
    A("### 3. 判断の記録（各ブランチの `agent/DECISIONS.md` の見出し。重複は 1 つ）\n")
    heads: dict[str, list[str]] = defaultdict(list)
    for b in sorted(bs):
        txt = git("show", f"{b}:agent/DECISIONS.md", check=False)
        for m in re.finditer(r"^## (20\d\d-\d\d-\d\d) — (.+)$", txt, re.M):
            key = f"{m.group(1)} — {m.group(2).strip()}"
            if b not in heads[key]:
                heads[key].append(b)
    for key in sorted(heads):
        A(f"- {key}（{', '.join(f'`{x}`' for x in heads[key])}）")
    A("")

    # 4. ENTRY の索引
    sh = ai_shared()
    A("### 4. Design ↔ Engineering の ENTRY の索引（`ai-shared/integration-log.md`。git 管理外）\n")
    if sh and (sh / "integration-log.md").exists():
        txt = (sh / "integration-log.md").read_text(encoding="utf-8")
        ents = []
        for m in re.finditer(r"^(ENTRY-[A-Z]+-\d+)\s*\nFrom: ([^\n]+)\n(?:To: ([^\n]+)\n)?Area: ([^\n]+)", txt, re.M):
            ents.append((m.group(1), m.group(2).split("（")[0].strip(), m.group(4).strip()))
        ents.sort(key=lambda e: (e[0].split("-")[1], int(e[0].split("-")[2])))
        for eid, frm, area in ents:
            A(f"- **{eid}**（{frm}）{area[:150]}")
        A(f"\n（{len(ents)} 件）")
    else:
        A("（`ai-shared/integration-log.md` が見つからない）")
    A("")
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    text = OUT.read_text(encoding="utf-8")
    if BEGIN not in text or END not in text:
        print(f"{OUT} に {BEGIN} / {END} が無い", file=sys.stderr)
        return 2
    i, j = text.index(BEGIN) + len(BEGIN), text.index(END)
    new = "\n" + build() + "\n"
    if a.check:
        ok = text[i:j] == new
        print("最新" if ok else "古い（python tools/update_history.py で更新）")
        return 0 if ok else 1
    OUT.write_text(text[:i] + new + text[j:], encoding="utf-8", newline="\n")
    print(f"更新: {OUT}（{len(new.splitlines())} 行）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
