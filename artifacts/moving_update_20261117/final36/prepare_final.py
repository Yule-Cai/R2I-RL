"""Freeze a complete final evaluation before observing any final-cohort outcome."""
import copy
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'ras_moving_supplement_20261117'
QUALIFIED = ROOT.parent / 'ras_relay_diagnostic_20261117'
REMOTE = '/home/gamer/Projects/rulesync-ras-final-moving36-20261117'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    assert not (ROOT / 'manifest.json').exists()
    for directory in ('source', 'baseline', 'inputs', 'configs', 'analysis'):
        origin = QUALIFIED if directory == 'source' else PARENT
        target = ROOT / directory
        target.mkdir()
        for p in (origin / directory).iterdir():
            if p.is_file() and p.suffix in ('.py', '.json', '.csv', '.diff'):
                shutil.copy2(p, target / p.name)
    shutil.copy2(QUALIFIED / 'test_relay_zero_delay.py', ROOT / 'test_relay_zero_delay.py')
    for name in ('campaign.py', 'test_campaign_contract.py', 'decode_all.py',
                 'aggregate_campaign.py', 'export_review_artifact.py'):
        shutil.copy2(PARENT / name, ROOT / name)
    name = 'collect_and_audit.py'
    text = (PARENT / name).read_text()
    text = text.replace('/home/gamer/Projects/rulesync-ras-moving-20261117', REMOTE)
    (ROOT / name).write_text(text)
    # Export keeps this preparation script; it must never prepare another cohort.
    name = 'export_review_artifact.py'
    text = (ROOT / name).read_text().replace("'prepare_campaign.py',", "'prepare_final.py', 'test_relay_zero_delay.py', 'qualified_source_comparison.json',")
    (ROOT / name).write_text(text)
    original = json.loads((PARENT / 'manifest.json').read_text())
    manifest = copy.deepcopy(original)
    manifest.update(
        campaign='RAS_final_moving_update_36_20261117',
        purpose='Complete final evaluation with a qualified response-forwarding harness',
        evaluation_stage='final_evaluation_after_harness_qualification',
        original_manifest_sha256=sha(PARENT / 'manifest.json'),
        qualified_diagnostic_manifest_sha256=sha(QUALIFIED / 'manifest.json'),
        original_results_retained=True,
        cohort_reporting='This complete cohort is designated for final evaluation before its outcomes; development and qualification records are retained separately.',
        source_change='Only zero-delay forwarding in the test relay differs from the development 36-run cohort. The production core is unchanged.',
        resource_policy=dict(minimum_available_memory_gib=5, minimum_free_disk_gib=30,
                             wait_for_observed_graphics_program_to_exit=True,
                             workers=1, automatic_retry=False))
    changes = []
    for p in sorted((ROOT / 'source').glob('*.py')):
        prior = PARENT / 'source' / p.name
        assert p.read_bytes() == (QUALIFIED / 'source' / p.name).read_bytes()
        if p.read_bytes() != prior.read_bytes():
            changes.append(p.name)
    assert changes == ['snapshot_response_proxy_v1.py'], changes
    comparison = dict(changed_from_development=changes,
                      final_source_matches_qualified_diagnostic=True,
                      original_manifest_sha256=manifest['original_manifest_sha256'],
                      qualified_manifest_sha256=manifest['qualified_diagnostic_manifest_sha256'])
    (ROOT / 'qualified_source_comparison.json').write_text(json.dumps(comparison, indent=2)+'\n')
    manifest['inputs_sha256'] = {
        str(p.relative_to(ROOT)): sha(p)
        for directory in ('source', 'baseline', 'inputs', 'configs')
        for p in sorted((ROOT / directory).rglob('*')) if p.is_file()}
    for name in ('prepare_final.py', 'campaign.py', 'test_campaign_contract.py',
                 'test_relay_zero_delay.py', 'qualified_source_comparison.json',
                 'decode_all.py', 'collect_and_audit.py', 'aggregate_campaign.py',
                 'export_review_artifact.py', 'run_final_when_ready.py', 'PROTOCOL_ZH.md'):
        manifest['inputs_sha256'][name] = sha(ROOT / name)
    (ROOT / 'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(dict(planned_runs=36, planned_pairs=18,
                         manifest_sha256=sha(ROOT/'manifest.json'),
                         changed_source=changes)))


if __name__ == '__main__':
    main()
