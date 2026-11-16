"""Capture primary Crossref metadata for a manuscript's cited DOI entries.

This records metadata/existence, not claim support or exhaustive retraction status.
"""
import argparse
import concurrent.futures
import datetime
import json
import re
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--bib", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    source = args.source.read_text()
    citations = list(dict.fromkeys(
        key.strip() for group in re.findall(r"\\cite\{([^}]+)\}", source)
        for key in group.split(",")))
    entries = {}
    for kind, key, body in re.findall(
            r"@(\w+)\{([^,]+),\s*(.*?)(?=\n\}\s*(?:\n|$))",
            args.bib.read_text(), re.S):
        fields = dict(re.findall(r"^\s*(\w+)\s*=\s*\{(.*)\},?\s*$", body, re.M))
        entries[key] = {"kind": kind, "fields": fields}
    missing = set(citations) - set(entries)
    if missing:
        raise ValueError(f"Missing bibliography keys: {sorted(missing)}")

    def fetch(key):
        row = {"key": key, "local": entries[key]}
        doi = entries[key]["fields"].get("doi")
        if not doi:
            row["status"] = "needs_primary_web_check"
            return row
        url = "https://api.crossref.org/works/" + urllib.parse.quote(doi, safe="")
        row["metadata_url"] = url
        request = urllib.request.Request(url, headers={
            "User-Agent": "RuleSync-Bibliography-Audit/1.0",
            "Accept": "application/json"})
        try:
            with urllib.request.urlopen(request, timeout=30) as response:
                metadata = json.load(response)["message"]
            row["status"] = "crossref_found"
            row["primary_metadata"] = {
                name: metadata.get(name) for name in [
                    "DOI", "title", "author", "container-title", "volume",
                    "issue", "page", "published", "published-print",
                    "published-online", "abstract", "URL", "link",
                    "update-to", "relation", "publisher",
                ]}
        except (urllib.error.URLError, TimeoutError, ValueError) as error:
            row["status"] = "metadata_unavailable"
            row["error"] = str(error)
        return row

    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        rows = list(pool.map(fetch, citations))
    result = {
        "checked_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "cited_keys": citations,
        "unused_bib_keys": sorted(set(entries) - set(citations)),
        "rows": rows,
        "scope": "Primary metadata only; abstract/claim-support checks are separate.",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    for row in rows:
        metadata = row.get("primary_metadata", {})
        print(row["key"], row["status"], metadata.get("title"),
              metadata.get("volume"), metadata.get("page"),
              metadata.get("published"))


if __name__ == "__main__":
    main()
