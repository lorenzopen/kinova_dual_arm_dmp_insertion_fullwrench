"""Read actual cart_d metadata and source-specific force metrics; no causal A/B claim."""
import json
from collections import defaultdict
from pathlib import Path

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[2]
REVIEW=OUT.parent/'archive_review'


def main():
    records=json.loads((REVIEW/'run_catalog.json').read_text())
    metrics={m['id']:m for m in json.loads((REVIEW/'successful_and_reference_metrics.json').read_text())}
    groups=defaultdict(list);rows=[]
    for rec in records:
        meta=rec.get('meta',{});params=meta.get('controller_parameters') or {}
        if not all(params.get(side,{}).get('cart_d') for side in 'LR'):continue
        execution=meta.get('execution') or {};row={k:rec[k] for k in ['id','source','metadata','piece','status','label']}
        row.update(cart_d={s:params[s]['cart_d'] for s in 'LR'},
                   cart_k={s:params[s].get('cart_k') for s in 'LR'},
                   controller_mode=execution.get('mode'),squeeze_gain=execution.get('squeeze_gain_scale'),
                   channels=execution.get('wrench_channels'),
                   ffi=execution.get('ffi'),result=meta.get('result'),git=meta.get('git'))
        if rec['id'] in metrics:
            m=metrics[rec['id']]
            row['measured']={k:m[k] for k in ['duration_s','tracking_rmse_mean_hand_N','bands_mean_hand']}
        rows.append(row);groups[str(row['cart_d'])].append(row['id'])
    out=dict(note='Only runtime cart_d metadata counts as confirmed. Missing metadata is unknown, not D=60. No matched damping-only FFI experiment exists among these records.',
             groups=[dict(cart_d=k,count=len(v),ids=v) for k,v in groups.items()],runs=rows)
    (OUT/'damping_metadata_audit.json').write_text(json.dumps(out,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    print('Confirmed damping regimes:',[(k,len(v)) for k,v in groups.items()])
    for r in rows:
        if r['cart_d']['L'][0]==80:print(json.dumps(r),flush=True)


if __name__=='__main__':main()
