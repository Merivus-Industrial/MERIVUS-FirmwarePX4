#!/usr/bin/env python3
"""Index raw AFCR trial JSON and ULog paths with SHA-256 before archiving."""

import argparse
import hashlib
import json
from pathlib import Path


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    args = parser.parse_args()
    run = args.run.resolve()
    files = sorted((path for path in run.rglob("*") if path.is_file()
                    and path.suffix in (".json", ".ulg")
                    and path.name != "evidence_index.json"), key=lambda path: str(path))
    index = [{"relative_path": str(path.relative_to(run)), "absolute_path": str(path),
              "bytes": path.stat().st_size, "sha256": sha256(path)} for path in files]
    (run / "evidence_index.json").write_text(json.dumps(index, indent=2, sort_keys=True) + "\n",
                                              encoding="utf-8")
    print(f"indexed {len(index)} raw JSON/ULog files")


if __name__ == "__main__":
    main()
