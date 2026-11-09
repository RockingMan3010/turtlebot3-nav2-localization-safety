#!/usr/bin/env python3
"""Check relative Markdown links in this repository; no network required."""
from pathlib import Path
from urllib.parse import unquote
import re
ROOT=Path(__file__).resolve().parents[1]
bad=[]
checked=0
for doc in sorted(ROOT.rglob("*.md")):
    if ".git" in doc.parts: continue
    content=doc.read_text(encoding="utf-8")
    for url in re.findall(r"!?\[[^\]]*\]\(([^)]*)\)",content):
        url=url.strip().split()[0].strip("<>")
        if not url or url.startswith(("http://","https://","mailto:","#","data:")): continue
        target=unquote(url.split("#",1)[0].split("?",1)[0])
        checked+=1
        if not (doc.parent/target).exists():
            bad.append((str(doc.relative_to(ROOT)),target))
if bad:
    for doc,target in bad: print("BROKEN",doc,"->",target)
    raise SystemExit(f"FAIL: {len(bad)} broken relative Markdown links")
print(f"PASS: {checked} relative Markdown links resolve in the curated repository")
