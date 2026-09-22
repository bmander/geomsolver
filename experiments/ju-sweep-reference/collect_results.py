"""Retain compact case evidence and produce a Markdown measurement table."""
import argparse
import json
from pathlib import Path
import shutil


def verdict(audit,encoding):
    t=audit['topology']; e=audit[encoding]
    if any(t[k] for k in ['boundary_edges','nonmanifold_edges','inconsistent_edges','nonmanifold_vertex_links']):
        return 'FAIL topology'
    if e['degenerate_triangles'] or e['coordinate_aliases'] or e['improper_pairs']:
        return 'FAIL geometry'
    return 'pass' if e['pair_scan_complete'] else 'incomplete scan'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('results',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--prefix',default='',help='Prefix aggregate filenames, e.g. prism-')
    args=parser.parse_args(); args.output.mkdir(parents=True,exist_ok=True)
    records=[]
    lines=['| Case | Run seconds | Triangles | Float64 mesh | STL mesh | Sampled forward error | Volume error |',
           '| --- | ---: | ---: | --- | --- | ---: | ---: |']
    for path in sorted(args.results.glob('*/result.json')):
        r=json.loads(path.read_text()); records.append(r)
        dest=args.output/path.parent.name; dest.mkdir(exist_ok=True)
        for filename in ['result.json','sweep.yaml','config.yaml','run.log','tool.obj','tool-at-start.stl']:
            if not (path.parent/filename).exists(): continue
            target=dest/filename
            if filename=='result.json' and target.exists() and (dest/'swept-solid.stl').exists():
                previous=json.loads(target.read_text()).get('audit',{}).get('sha256')
                current=r.get('audit',{}).get('sha256')
                if previous and current and previous!=current:
                    raise ValueError(f'Measurement would no longer match delivered STL: {dest}')
            shutil.copyfile(path.parent/filename,target)
        a=r.get('audit')
        if not a:
            status=r.get('audit_error','audit pending' if r['status']=='ok' else r['status'])
            lines.append(f"| {r['case']} | {r['run_seconds']:.2f} | — | {status} | — | — | — |")
            continue
        c=a.get('capsule'); error=f"{c['max_mesh_to_capsule']:.3g}" if c else '—'
        volume=f"{100*(a['topology']['signed_volume']/c['analytic_volume']-1):+.4f}%" if c else '—'
        lines.append(f"| {r['case']} | {r['run_seconds']:.2f} | {a['topology']['triangles']:,} | {verdict(a,'binary64')} | {verdict(a,'binary32')} | {error} | {volume} |")
    (args.output/(args.prefix+'results.json')).write_text(json.dumps(records,indent=2)+'\n')
    (args.output/(args.prefix+'measurements.md')).write_text('\n'.join(lines)+'\n')
    print('\n'.join(lines))


if __name__=='__main__': main()
