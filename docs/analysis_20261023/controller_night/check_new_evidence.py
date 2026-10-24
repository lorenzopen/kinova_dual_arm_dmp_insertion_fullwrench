"""Bounded intake check after completed offline studies; no repeated simulations.

Archive scan compares path/size/mtime only. Full SHA preservation remains a
separate final-delivery action; this script must never claim to replace it.
"""
import sys
sys.dont_write_bytecode=True
import hashlib
import json
from pathlib import Path
from datetime import datetime,timezone

OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[2]


def read(path):return json.loads(path.read_text(encoding='utf-8'))
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    now=datetime.now(timezone.utc).isoformat();state_path=OUT/'STATE.json';state=read(state_path)
    before=read(OUT.parent/'archive_review/archive_manifest_before.json')
    expected={Path(x['path']).as_posix():(x['bytes'],x['mtime_ns']) for x in before}
    source=ROOT/'NVIDIA_RUNS';actual={}
    for path in source.rglob('*'):
        if path.is_file():
            st=path.stat();actual[path.relative_to(source).as_posix()]=(st.st_size,st.st_mtime_ns)
    added=sorted(actual.keys()-expected.keys());removed=sorted(expected.keys()-actual.keys())
    changed=sorted(k for k in actual.keys()&expected.keys() if actual[k]!=expected[k])
    primary=read(OUT/'FINAL_COMPARISON_FREEZE.json');supplement=read(OUT/'FINAL_COMPARISON_SOURCE_SUPPLEMENT.json')
    fingerprints=primary['source_fingerprints']+supplement['additional_source_fingerprints']
    changed_sources=[e['path'] for e in fingerprints if not Path(e['path']).is_file() or sha(Path(e['path']))!=e['sha256']]
    checkpoint=OUT/state['latest_source_checkpoint']
    checkpoint_manifest=read(checkpoint.with_name(checkpoint.stem+'_manifest.json'))
    assert sha(checkpoint)==checkpoint_manifest['zip_sha256'],'Latest source checkpoint changed'
    # Repository root and known immediate package layouts only. Broad external
    # Desktop searches were already done and are not repeated without new input.
    candidates=[ROOT/'kinova_controller.py',ROOT/'kinova_control/kinova_controller.py',
        ROOT/'kinova_controller_pkgs/kinova_control/kinova_control/kinova_controller.py',
        ROOT/'src/kinova_control/kinova_control/kinova_controller.py']
    discovered=[p.relative_to(ROOT).as_posix() for p in candidates if p.is_file()]
    needs_review=bool(added or removed or changed or changed_sources or discovered)
    result=dict(checked_utc=now,kind='bounded_new_evidence_intake',archive_files=len(actual),
        archive_metadata_matches_original=not(added or removed or changed),
        archive_added=added,archive_removed=removed,archive_metadata_changed=changed,
        archive_full_SHA_reverified_this_check=False,
        last_full_preservation=read(OUT/'preservation_verification.json'),
        frozen_source_files_checked=len(fingerprints),changed_frozen_sources=changed_sources,
        inner_servo_sources_in_known_layouts=discovered,latest_checkpoint_sha_verified=True,
        new_evidence_or_change_requires_review=needs_review,
        action='Review new evidence without changing frozen split or tuning on holdout' if needs_review else
            'No new discriminating input. Keep conclusions/parameters frozen; no repeated grids, payload rebuild or unchanged-status notification.',
        limitation='Path/size/mtime scan is not a fresh full-content SHA verification. Source absence check covers known layouts, not every user directory.')
    (OUT/'EVIDENCE_WATCH.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    state['last_evidence_check']=dict(checked_utc=now,archive_metadata_unchanged=not(added or removed or changed),
        frozen_sources_unchanged=not changed_sources,new_servo_source=bool(discovered),
        requires_review=needs_review,full_archive_SHA_reverified=False)
    state_path.write_text(json.dumps(state,indent=2)+'\n',encoding='utf-8')
    with (OUT/'NIGHT_LOG.md').open('a',encoding='utf-8') as f:
        f.write(f'\n## Intake {now}\n'
            f'Letti istruzioni/stato/registro/report e steering damping; nessun Python persistente. '
            f'Controllo bounded: {len(actual)} file NVIDIA, aggiunti {len(added)}, rimossi {len(removed)}, '
            f'metadata cambiati {len(changed)}; {len(fingerprints)} fingerprint sorgenti SHA, '
            f'cambiati {len(changed_sources)}. Checkpoint piu recente SHA verificato. '
            f'Sorgente servo nei layout noti: {len(discovered)}. '
            f'Nessun nuovo sweep/fit/tuning/payload; archivio solo in lettura. '
            f'Fast check size/mtime NON sostituisce verifica SHA completa alla chiusura. '
            f'Nuova evidenza da riesaminare: {needs_review}.\n')
    print(json.dumps({k:result[k] for k in ['archive_files','archive_metadata_matches_original',
        'changed_frozen_sources','inner_servo_sources_in_known_layouts','new_evidence_or_change_requires_review']},indent=2))


if __name__=='__main__':main()
