"""Builds docs/walkthrough/*.md from docs/walkthrough/_src/*.md templates.

Run from anywhere:  python docs/walkthrough/_src/build.py

Why: snippets are copied from the real files by anchor text, so a rebuild refreshes both the
code and its `path:line` references after the code changes. If an anchor no longer exists the
build stops and names it -- that is the signal that a chapter needs rewriting.

Template markers (each on a line of its own):

  {{snip: path :: start => end :: start2 => end2 +N}}
      One code block. Each range starts at the first line (after the previous range) that
      CONTAINS `start`, and ends at the first line at/after it that contains `end` (omit
      `=> end` for a single line). `+N` adds N more lines after the end line. Several
      ranges in one block are joined with "..." and the header lists every line range.
      Prefix a range with `@` to search from the top of the file instead of after the
      previous range (`@start => end`).
  {{run: demos/name.py}}
      Runs the script from the repo root and inserts its output as a text block.
"""

import pathlib
import re
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
OUT = HERE.parent
ROOT = OUT.parent.parent

LANG = {".py": "python", ".json": "json", ".md": "markdown"}


def find(lines, text, begin):
    for i in range(begin, len(lines)):
        if text in lines[i]:
            return i
    return None


def snippet(spec):
    parts = [p.strip() for p in spec.split("::")]
    path = parts[0]
    src = (ROOT / path).read_text(encoding="utf-8").splitlines()
    out, ranges, cursor = [], [], 0
    for rng in parts[1:]:
        extra = 0
        m = re.search(r"\s\+(\d+)$", rng)
        if m:
            extra, rng = int(m.group(1)), rng[: m.start()]
        top = rng.startswith("@")
        rng = rng[1:].strip() if top else rng
        start_txt, _, end_txt = rng.partition("=>")
        start_txt, end_txt = start_txt.strip(), end_txt.strip()
        a = find(src, start_txt, 0 if top else cursor)
        if a is None:
            raise SystemExit(f"ANCHOR NOT FOUND in {path}: {start_txt!r}")
        b = a
        if end_txt:
            b = find(src, end_txt, a)
            if b is None:
                raise SystemExit(f"END ANCHOR NOT FOUND in {path}: {end_txt!r} (after {start_txt!r})")
        b = min(b + extra, len(src) - 1)
        if out:
            out.append("    ...")
        out.extend(src[a : b + 1])
        ranges.append(f"{a + 1}-{b + 1}" if b > a else f"{a + 1}")
        cursor = b + 1
    lang = LANG.get(pathlib.Path(path).suffix, "")
    header = f"# {path}:{', '.join(ranges)}"
    return f"```{lang}\n{header}\n" + "\n".join(out) + "\n```"


def run(script):
    p = subprocess.run([sys.executable, str(HERE / script)], cwd=ROOT, capture_output=True,
                       text=True, encoding="utf-8", timeout=240)
    text = (p.stdout + (("\n[stderr]\n" + p.stderr) if p.returncode else "")).rstrip()
    if p.returncode:
        print(f"WARNING: {script} exited {p.returncode}", file=sys.stderr)
    return "```text\n" + text + "\n```"


def build(tpl):
    text = tpl.read_text(encoding="utf-8")

    def sub(m):
        kind, arg = m.group(1), m.group(2).strip()
        return snippet(arg) if kind == "snip" else run(arg)

    return re.sub(r"^\{\{(snip|run):(.*?)\}\}[ \t]*$", sub, text, flags=re.M)


if __name__ == "__main__":
    for tpl in sorted(HERE.glob("*.md")):
        (OUT / tpl.name).write_text(build(tpl), encoding="utf-8", newline="\n")
        print("built", tpl.name)
