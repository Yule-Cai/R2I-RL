"""Paper sim evaluation analyzer: CSVs -> markdown tables + JSON summary.

Inputs: bench CSVs (headless planner matrix), driver CSVs (physics),
fault CSVs, endurance CSV. Outputs summary.json + TABLES.md (pasted
into reports/PAPER_EVALUATION_SIMULATION.md).

Usage:
  python3 -m topopatch.tools.analyze_paper --bench 'bench_*.csv' \
      --driver 'driver_*.csv' --faults 'fault_*.csv' \
      --endurance endurance.csv --out <dir>
"""
import argparse
import csv
import glob
import json
import os
from collections import defaultdict


def read_csvs(patterns):
    rows = []
    for pat in patterns:
        for fp in glob.glob(pat):
            with open(fp) as f:
                for r in csv.DictReader(f):
                    r["_file"] = os.path.basename(fp)
                    rows.append(r)
    return rows


def f(x, d=3):
    try:
        return round(float(x), d)
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bench", nargs="*", default=[])
    ap.add_argument("--driver", nargs="*", default=[])
    ap.add_argument("--faults", nargs="*", default=[])
    ap.add_argument("--endurance", default=None)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    out = {"planner_matrix": {}, "physics": {}, "faults": {},
           "endurance": {}}
    lines = ["# Paper sim evaluation tables (generated)",
             f"Source: freeze {a.out}"]

    bench = read_csvs(a.bench)
    by_pm = defaultdict(list)
    for r in bench:
        by_pm[(r.get("map", "?"), r.get("planner", "?"))].append(r)
    lines.append("\n## Headless planner matrix (repair, no execution)")
    lines.append("| map | planner | n | plan_ok | accept | abstain-nonlocal "
                 "| allfail | fallback_ok | viol_exec | commit->exec_ms_med |")
    for (mp, pl), rs in sorted(by_pm.items()):
        n = len(rs)
        pok = sum(1 for r in rs if r.get("plan_result") == "ok") / n
        acc = sum(1 for r in rs if r.get("accepted") == "True") / n
        nonloc = sum(1 for r in rs
                     if "nonlocal" in str(r.get("reason", ""))) / n
        af = sum(1 for r in rs if str(r.get("reason", "")).startswith(
            "all-fail")) / n
        fb = [r for r in rs if r.get("fallback_ok") == "True"]
        fbok = len(fb) / max(1, sum(
            1 for r in rs if r.get("fallback") == "True"))
        lats = sorted(f(r.get("commit_to_executable_ms"))
                      for r in rs if f(r.get("commit_to_executable_ms"))
                      is not None)
        med = lats[len(lats) // 2] if lats else None
        out["planner_matrix"][f"{mp}/{pl}"] = {
            "n": n, "plan_ok": round(pok, 3), "accept": round(acc, 3),
            "abstain_nonlocal": round(nonloc, 3), "allfail": round(af, 3),
            "fallback_ok": round(fbok, 3),
            "commit_to_executable_ms_med": med}
        lines.append(f"| {mp} | {pl} | {n} | {pok:.2f} | {acc:.2f} | "
                     f"{nonloc:.2f} | {af:.2f} | {fbok:.2f} | 0 | {med} |")

    drv = read_csvs(a.driver)
    by_dc = defaultdict(list)
    for r in drv:
        by_dc[(r.get("map", "?"), r.get("planner", "?"),
               r.get("controller", r.get("exec_mode", "?")))].append(r)
    lines.append("\n## Closed-loop physics (execution)")
    lines.append("| map | planner | ctrl | n | arrived | rule_compl | "
                 "coll_contacts | path_len_med | t_goal_med | fail_modes |")
    for (mp, pl, ct), rs in sorted(by_dc.items()):
        n = len(rs)
        arr = sum(1 for r in rs if r.get("arrived") == "True") / n
        rc = sum(1 for r in rs if r.get("rule_compliant") == "True") / n
        cc = sum(int(float(r.get("scan_contacts") or 0)) for r in rs)
        pls = sorted(v for v in
                     (f(r.get("plan_len") or r.get("fallback_len"))
                      for r in rs) if v is not None)
        tts = sorted(v for v in (f(r.get("exec_t")) for r in rs)
                     if v is not None)
        fails = defaultdict(int)
        for r in rs:
            if r.get("arrived") != "True":
                fails[str(r.get("reason", "?"))[:40]] += 1
        out["physics"][f"{mp}/{pl}/{ct}"] = {
            "n": n, "arrived": round(arr, 3), "rule_compliant": round(rc, 3),
            "scan_contacts": cc,
            "path_len_med": pls[len(pls) // 2] if pls else None,
            "t_goal_med": tts[len(tts) // 2] if tts else None,
            "fail_modes": dict(fails)}
        lines.append(
            f"| {mp} | {pl} | {ct} | {n} | {arr:.2f} | {rc:.2f} | {cc} | "
            f"{pls[len(pls)//2] if pls else None} | "
            f"{tts[len(tts)//2] if tts else None} | {dict(fails)} |")

    flt = read_csvs(a.faults)
    lines.append("\n## Fault injection (fail-closed + recovery)")
    lines.append("| fault | fail_closed | motion | stop/reject_lat_s | "
                 "recovery_lat_s | note |")
    for r in flt:
        out["faults"][r.get("fault", "?")] = {
            k: r.get(k) for k in
            ("fail_closed", "motion", "stop_latency_s",
             "reject_latency_s", "timeout_latency_s", "fail_latency_s",
             "block_latency_s", "recovery_latency_s", "recovered",
             "violator_blocked", "clean_forwarded", "commit_ok",
             "plan_ok")}
        lines.append(
            f"| {r.get('fault')} | {r.get('fail_closed')} | "
            f"{r.get('motion')} | {r.get('stop_latency_s', r.get('reject_latency_s', r.get('timeout_latency_s', r.get('fail_latency_s', r.get('block_latency_s')))))} | "
            f"{r.get('recovery_latency_s')} | epoch={r.get('epoch')} |")

    if a.endurance and os.path.exists(a.endurance):
        with open(a.endurance) as fh:
            erows = list(csv.DictReader(fh))
        rss = defaultdict(list)
        for r in erows:
            for k in r:
                if k.startswith("rss_"):
                    try:
                        rss[k].append(int(r[k]))
                    except Exception:
                        pass
        mem = {k: {"first": v[0], "max": max(v), "last": v[-1],
                   "growth_kb": max(v) - v[0]} for k, v in rss.items()
               if v}
        anoms = sum(1 for r in erows if r.get("missed_verifier") == "True")
        stale = sum(1 for r in erows if (r.get("list_active") not in
                                         ("0", None, "")))
        out["endurance"] = {"iters": len(erows), "mem_kb": mem,
                            "missed_verifier": anoms,
                            "stale_iters": stale}
        lines.append("\n## Endurance (60 min)")
        lines.append(f"iters={len(erows)} missed_verifier={anoms} "
                     f"stale_iters={stale}")
        for k, v in mem.items():
            lines.append(f"- {k}: first={v['first']}KB max={v['max']}KB "
                         f"growth={v['growth_kb']}KB")
    json.dump(out, open(os.path.join(a.out, "paper_summary.json"),
                        "w"), indent=1)
    open(os.path.join(a.out, "TABLES.md"), "w").write("\n".join(lines))
    print("\n".join(lines))


if __name__ == "__main__":
    main()
