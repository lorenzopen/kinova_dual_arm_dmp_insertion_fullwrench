"""Seal new simulation evidence only. Original preservation checked separately."""
import json,hashlib
from datetime import datetime,timezone
from pathlib import Path
OUT=Path(__file__).resolve().parent;GEN=OUT/'gen3'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    audit=json.loads((OUT/'input_integrity_and_profile_parity.json').read_text())
    assert audit['NVIDIA_originals_identical'] and audit['baseline_sha256_verified']
    assert audit['NVIDIA_files']==765 and audit['unchanged_runtime_files']==332
    verify=json.loads((GEN/'lab_verification.json').read_text())
    assert all(verify['reference']['executable_method_AST_equal'].values())
    source=json.loads((GEN/'lab_source/provenance.json').read_text())
    assert sha(Path(source['source']))==sha(GEN/'lab_source/kinova_controller_original.py')==source['source_sha256']
    state=dict(updated_utc=datetime.now(timezone.utc).isoformat(),worker_pid=None,
        status='original_controller_integrated_plant_validation_failed',
        achieved=['Two articulated Gen3 in recorded nominal lab transforms','Central2700 + three500 springs',
            'Original FFI and offline SOCP','Verbatim Cartesian/Kalman classes with real Pinocchio',
            'User-supplied compute/estimate methods executable-AST matched','Kortex reaction torque sign and250Hz estimate/pose publishing',
            'Independent FK/J/gravity, mechanical FT static, slew/sign checks','Preserved all failures and all originals'],
        not_validated=['Full insertion prediction with original impedance','Actuator/transmission/physical joint dynamics','Deployed entire-node file hash'],
        current_results='LAB_IMPEDANCE_RESULTS.md',
        latest_trials=[x['run'] for x in verify['latest_cases']],
        next_required='Validate nominal stationary hold and small free-space motion against actual joint torque/state/timing before insertion claims',
        originals=dict(NVIDIA_files=audit['NVIDIA_files'],NVIDIA_bytes=audit['NVIDIA_bytes'],runtime_files=audit['unchanged_runtime_files'],sha256_verified=True))
    (GEN/'STATE.json').write_text(json.dumps(state,indent=2)+'\n')
    log='''# Registro finale della nuova prova PyBullet\n\n- Richiesta iniziale: stesso FFI/SOCP autonomo; poi centrale2700N/m, poi due Gen3 nella geometria del laboratorio.\n- Primo backend flottante: Z/L/S/T e raffinamenti; central2700 implica4200N/m totali. Interpretazione precedente totale2700 superata e conservata.\n- Gen3 articolati: mesh ufficiali e URDF locale, trasformazioni del lab e FK verificati. Rimossa nella sola copia la massa predefinita Bullet1kg di un frame privo di inerzia.\n- Servo approssimato: esplicito instabile; implicito nominale completa Z/L, ma Z cambia77.2→4.4mm col raffinamento. Risultato preliminare, nessuna attribuzione hardware.\n- Nuovo steering: trovato controller originale2092 righe nei Download. Snapshot immutato; classi estratte verbatim e Pinocchio isolato. Il codice inviato da Lorenzo conferma compute_torque/estimate_wrench mediante confronto AST.\n- Loop simulato: K/D originali, Kalman/RC, clip35/12, slew0.5, watchdog2rad/s20tick, friction scale0, torque_sensor=-tau_applied precedente, pubblicazione250Hz e ATI separati. Homing non simulato; covarianza fresca/assestata dichiarate.\n- La legge originale sul plant URDF abortisce già in aria. Verificati FK/J/gravità, assenza collisioni iniziali, sensore in statica e segni. Pinocchio ABA indipendente senza contatti conferma crescita prima del watchdog a due dt, sia con Kalman fresco sia assestato.\n- Conclusione: integrazione controller realizzata; plant non ancora validato per giudicare l'inserimento. Nessun damping/inerzia arbitrari introdotti per ottenere una riuscita. Dati minimi mancanti nel report.\n- Nessun processo di simulazione lasciato attivo; nessun ROS/robot/deploy/push. NVIDIA, baseline e332 sorgenti runtime invariati con verifica SHA-256 finale.\n'''
    (OUT/'DELIVERY_LOG.md').write_text(log,encoding='utf-8')
    paths=[p for p in OUT.iterdir() if p.is_file() and p.name!='DELIVERY_MANIFEST.json']
    for folder in ['gen3','vendor','runs']:
        paths.extend(p for p in (OUT/folder).rglob('*') if p.is_file() and '__pycache__' not in p.parts)
    entries=[dict(path=p.relative_to(OUT).as_posix(),bytes=p.stat().st_size,sha256=sha(p)) for p in sorted(set(paths))]
    manifest=dict(created_utc=datetime.now(timezone.utc).isoformat(),files=entries,
        excludes='deps binary/runtime packages, scratch generated meshes, matplotlib cache; runtime package SHA records retained in provenance',
        input_integrity='input_integrity_and_profile_parity.json')
    (OUT/'DELIVERY_MANIFEST.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(dict(scientific_files=len(entries),bytes=sum(x['bytes'] for x in entries),state=state),indent=2))
if __name__=='__main__':main()
