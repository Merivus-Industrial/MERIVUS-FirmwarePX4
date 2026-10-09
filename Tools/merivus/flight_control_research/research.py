#!/usr/bin/env python3
"""Collect research metadata; full-text claims require separate source review."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
from urllib.parse import urlencode
from urllib.request import Request, urlopen


TOPICS = {
    "INDI": "quadrotor incremental nonlinear dynamic inversion disturbance rejection",
    "L1 adaptive": "quadrotor L1 adaptive control wind disturbance",
    "H infinity": "quadrotor H infinity robust controller online identification",
    "DOB ESO": "quadrotor disturbance observer extended state observer hover",
    "sliding mode": "quadrotor incremental sliding mode disturbance observer",
    "MPC": "quadrotor model predictive control wind disturbance hover",
    "adaptive control": "quadrotor adaptive control payload variation hover",
    "fault tolerance": "quadrotor fault tolerant control allocation",
    "online identification": "quadrotor online system identification controller",
    "control allocation": "multirotor control allocation actuator saturation",
    "residual learning": "quadrotor residual learning dynamics model predictive control",
}


def normalize_work(work, topic, fetched_at):
    title = work.get("title", [""])
    doi = work.get("DOI", "")
    if not title or not title[0] or not doi:
        return None
    return {
        "topic": topic,
        "title": title[0],
        "doi": doi.lower(),
        "url": "https://doi.org/" + doi,
        "publisher": work.get("publisher", ""),
        "published": work.get("published", {}).get("date-parts", [[None]])[0],
        "fetched_at_utc": fetched_at,
        "review_status": "METADATA_ONLY",
    }


def collect(rows=5):
    fetched_at = datetime.now(timezone.utc).isoformat()
    works = {}
    for topic, query in TOPICS.items():
        url = "https://api.crossref.org/works?" + urlencode({"query": query, "rows": rows,
                                                                "select": "DOI,title,publisher,published"})
        request = Request(url, headers={"User-Agent": "MERIVUS-flight-control-research/0.1 (research metadata)"})
        with urlopen(request, timeout=20) as response:
            items = json.load(response)["message"]["items"]
        for item in items:
            normalized = normalize_work(item, topic, fetched_at)
            if normalized:
                works[(topic, normalized["doi"])] = normalized
    return sorted(works.values(), key=lambda work: (work["topic"], work["doi"]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--rows", type=int, default=5)
    args = parser.parse_args()
    if not 1 <= args.rows <= 20:
        raise ValueError("rows must be 1..20")
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.write_text(json.dumps(collect(args.rows), indent=2, ensure_ascii=False) + "\n",
                           encoding="utf-8")


if __name__ == "__main__":
    main()
