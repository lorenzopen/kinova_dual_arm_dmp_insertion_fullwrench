"""Seal derived deliverables and verify preservation of sources and all trials."""
from analyze import *
import platform

def main():
    assert not any((HERE/name).exists() for name in ['runs.lock','pilot.lock','refinement.lock'])
    manifest=json.loads((HERE/'runs_manifest.json').read_text());jobs=manifest['jobs']
    metrics=json.loads((HERE/'metrics.json').read_text());assert not metrics['exceptions']
    assert len(jobs)==len(metrics['records'])==111
    assert {j['id'] for j in jobs}=={r['id'] for r in metrics['records']}
    assert len({j['id'] for j in jobs})==111
    max_sign_error=0.;max_normal_error=0.;checked=0
    for job in jobs:
        folder=HERE/'runs'/job['id'];summary=json.loads((folder/'COMPLETED.json').read_text())
        assert summary['source_sha256']==manifest['source_sha256']
        d=dict(np.load(folder/'trace.npz',allow_pickle=False))
        assert np.all(np.diff(d['t'])>0) and d['force'].shape==(len(d['t']),6,3)
        assert np.isfinite(d['force']).all() and np.isfinite(d['object_pos']).all()
        loaded=d['normal_N']>.05
        comp=-np.sum(d['force']*d['normals'],axis=2)
        if loaded.any():
            max_sign_error=max(max_sign_error,float(np.abs(comp[loaded]-d['normal_N'][loaded]).max()))
            max_normal_error=max(max_normal_error,float(np.abs(np.linalg.norm(d['normals'],axis=2)[loaded]-1).max()))
        entries=np.load(folder/'manifold.npz')['entries']
        assert entries.shape[1]==13 and np.all(entries[:,2]>=0)
        checked+=1
    assert max_sign_error<1e-6 and max_normal_error<1e-9
    audit=json.loads((HERE/'AUDIT.json').read_text());sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
    assert all(sha(item['path'])==item['expected'] for item in audit['provenance'])
    assert sha(HERE/'inputs/prior_reference.py')==sha(ROOT/'kinova_dual_arm_dmp_insertion/prior.py')
    assert sha(HERE/'inputs/prior_torsional_C.npz')==manifest['prior_sha256']
    assert sha(HERE/'inputs/contact_geometry_snapshot.json')==json.loads((HERE/'capability.json').read_text())['source_sha256']
    assert json.loads((HERE/'REFINEMENT_STATE.json').read_text())['complete']
    # pypdf verifies the final report can be reopened; visual QA is separately recorded.
    from pypdf import PdfReader
    report=HERE/'output/pdf/studio_coni_floating_plate.pdf';pdf=PdfReader(report)
    assert len(pdf.pages)==12 and all(len(page.extract_text())>300 for page in pdf.pages)
    summary=dict(trials_checked=checked,failures_retained=sum(r['outcome']!='complete' for r in metrics['records']),
        no_exceptions=True,max_native_normal_force_sign_error_N=max_sign_error,max_contact_normal_unit_error=max_normal_error,
        source_hashes_match=True,report_pages=len(pdf.pages),report_sha256=sha(report),
        note='Only derived workspace outputs written. NVIDIA_RUNS and original papers/controllers untouched. See audit for source paths and hashes.')
    (HERE/'DELIVERY_CHECKS.json').write_text(json.dumps(summary,indent=2)+'\n')
    files=[]
    for path in sorted(HERE.rglob('*')):
        if not path.is_file() or any(part in ['tmp','mplconfig','__pycache__','scratch'] for part in path.relative_to(HERE).parts):continue
        if path.name=='SHA256.json':continue
        files.append(dict(path=path.relative_to(HERE).as_posix(),bytes=path.stat().st_size,sha256=sha(path)))
    seal=dict(files=files,originals=audit['provenance'],excluded='Ephemeral tmp/mplconfig/scratch and the checksum manifest itself; original PDFs are verified by absolute source hash.')
    (HERE/'SHA256.json').write_text(json.dumps(seal,indent=2)+'\n')
    assert all(sha(HERE/item['path'])==item['sha256'] for item in files)
    print(json.dumps(summary,indent=2));print('Manifest files verified:',len(files))

if __name__=='__main__':main()
