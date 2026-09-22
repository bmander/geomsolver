"""Run pinned upstream examples and controlled variants without changing its code."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import platform
import shutil
import subprocess
import time

import yaml
from audit import audit

COMMIT = '0bb260118443545029c98db2e98278fac27ba460'


def cases(source):
    def base(name, example):
        folder = source/'example'/example
        return dict(name=name, example=example,
                    sweep=yaml.safe_load((folder/'sweep.yaml').read_text()),
                    config=yaml.safe_load((folder/'config.yaml').read_text()))
    simple = base('simple-stock', 'simple')
    simple['capsule'] = dict(a=[-.5,0,0], b=[.5,0,0], radius=.2)
    yield simple
    for name in ['repeat','no-filter','coarse','fine','rotated','offset','small']:
        case = copy.deepcopy(simple); case['name'] = 'simple-'+name
        params = case['config']['parameters']
        if name == 'no-filter': params.update(volume_threshold=0.,face_count_threshold=0)
        if name in ('coarse','fine'):
            params.update(epsilon_env=2e-3 if name=='coarse' else 2.5e-4,
                          epsilon_sil=2e-3 if name=='coarse' else 2.5e-4)
        if name == 'rotated':
            axis = [1/math.sqrt(14),2/math.sqrt(14),3/math.sqrt(14)]
            case['capsule']['a'] = [-.5*x for x in axis]
            case['capsule']['b'] = [.5*x for x in axis]
        if name == 'offset':
            delta = [.013,.017,.019]
            for end in ('a','b'):
                case['capsule'][end] = [x+d for x,d in zip(case['capsule'][end],delta)]
        if name == 'small':
            for end in ('a','b'):
                case['capsule'][end] = [x*.1 for x in case['capsule'][end]]
            case['capsule']['radius'] *= .1
            for key in ('bbox_min','bbox_max'):
                case['config']['grid'][key] = [x*.1 for x in case['config']['grid'][key]]
            # Disable absolute cell filters and scale geometry tolerances and
            # the refinement length floor; the radius-ratio floor is dimensionless.
            params.update(epsilon_env=5e-5,epsilon_sil=5e-5,
                          min_tet_edge_length=2e-6,
                          volume_threshold=0.,face_count_threshold=0)
        case['sweep']['transform']['points'] = [case['capsule']['a'],case['capsule']['b']]
        case['sweep']['primitive']['radius'] = case['capsule']['radius']
        yield case
    for example in ['letter_L','flipping_torus']:
        case = base(example+'-stock',example); yield case
        case = copy.deepcopy(case); case['name'] = example+'-no-filter'
        case['config']['parameters'].update(volume_threshold=0.,face_count_threshold=0)
        yield case


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--case',action='append',help='Run only this case (repeatable)')
    parser.add_argument('--timeout',type=float,default=300)
    parser.add_argument('--audit-seconds',type=float,default=120)
    args = parser.parse_args()
    source,output = args.source.resolve(),args.output.resolve()
    head = subprocess.check_output(['git','-C',str(source),'rev-parse','HEAD'],text=True).strip()
    if head != COMMIT: raise SystemExit(f'Expected {COMMIT}, found {head}')
    binary = source/'build/generalized_sweep'
    if not binary.is_file(): raise SystemExit(f'Build {binary} first')
    output.mkdir(parents=True,exist_ok=True)
    metadata = dict(upstream='https://github.com/Jurwen/Swept-Volume',commit=head,
        source=str(source),platform=platform.platform(),machine=platform.machine(),
        python=platform.python_version(),binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
        source_diff=subprocess.check_output(['git','-C',str(source),'diff','HEAD'],text=True),
        timeout=args.timeout,audit_seconds=args.audit_seconds)
    (output/'environment.json').write_text(json.dumps(metadata,indent=2)+'\n')
    selected = list(cases(source))
    if args.case:
        unknown = set(args.case)-{c['name'] for c in selected}
        if unknown: raise SystemExit(f'Unknown cases: {sorted(unknown)}')
        selected = [c for c in selected if c['name'] in args.case]
    for case in selected:
        folder = output/case['name']
        if folder.exists():
            print(f"SKIP existing {folder}",flush=True); continue
        folder.mkdir()
        for key in ('sweep','config'):
            path = folder/(key+'.yaml')
            if case['name'].endswith('-stock') or case['name']=='simple-repeat':
                shutil.copyfile(source/'example'/case['example']/(key+'.yaml'),path)
            else: path.write_text(yaml.safe_dump(case[key],sort_keys=False))
        command = [str(binary),str(folder/'mesh'),'-f',str(folder/'sweep.yaml'),'-c',str(folder/'config.yaml')]
        result = dict(case=case['name'],command=command,cwd=str(folder),capsule=case.get('capsule'),
            generation_timeout=args.timeout,audit_seconds=args.audit_seconds,
            input_sha256={k:hashlib.sha256((folder/(k+'.yaml')).read_bytes()).hexdigest() for k in ('sweep','config')})
        print(f"START {case['name']}",flush=True)
        start = time.monotonic()
        with (folder/'run.log').open('w') as log:
            try:
                run = subprocess.run(command,cwd=folder,stdout=log,stderr=subprocess.STDOUT,timeout=args.timeout)
                result.update(status='ok' if run.returncode==0 else 'failed',returncode=run.returncode)
            except subprocess.TimeoutExpired: result['status'] = 'timeout'
        result['run_seconds'] = time.monotonic()-start
        (folder/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        if result['status']=='ok':
            try:
                result['audit'] = audit(folder/'mesh/sweep_surface.msh',case.get('capsule'),args.audit_seconds)
            except Exception as error:
                result['audit_error'] = repr(error)
        (folder/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(f"DONE {case['name']} {result['status']} {result['run_seconds']:.2f}s",flush=True)
    results = [json.loads(p.read_text()) for p in sorted(output.glob('*/result.json'))]
    (output/'results.json').write_text(json.dumps(results,indent=2)+'\n')


if __name__=='__main__': main()
