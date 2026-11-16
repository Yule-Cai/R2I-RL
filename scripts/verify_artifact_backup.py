#!/usr/bin/env python3
"""Read-only deterministic tree digest for exact two-host artifact comparison."""
import argparse
import hashlib
import json
from pathlib import Path


def digest_tree(root):
    tree = hashlib.sha256()
    count = size = 0
    for path in sorted(root.rglob("*")):
        if not path.is_file():
            continue
        if path.is_symlink():
            raise ValueError(f"artifact must contain actual files, not symlinks: {path}")
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024*1024), b""):
                digest.update(chunk)
                size += len(chunk)
        tree.update(path.relative_to(root).as_posix().encode()+b"\0"+digest.digest())
        count += 1
    if not count:
        raise ValueError(f"empty backup root: {root}")
    return dict(name=root.name, files=count, bytes=size, tree_sha256=tree.hexdigest())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("roots", nargs="+", type=Path)
    args = parser.parse_args()
    print(json.dumps([digest_tree(root) for root in args.roots]))


if __name__ == "__main__":
    main()
