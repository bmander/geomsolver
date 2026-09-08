#!/usr/bin/env python3
"""Audit complete-member material from explicit fields and generating motions.

Schema 5 independently establishes point classifications using full-roll
positive covers or attained removal/blank witnesses. The tight reported minimum
intervals, source error, whole-space coverage and export accuracy remain separate.
"""
from copy import deepcopy
from pathlib import Path
import json
import sys
import functional_sweep as f
import roll_cover


def encloses(declared,actual,label):
    assert declared[0] <= actual[0] <= actual[1] <= declared[1], (
        label,tuple(map(float,declared)),tuple(map(float,actual)))


def check(data,progress=False):
    assert data['schema'] in [4,5] and data['units'] == {'length':'mm','angle':'rad'}
    assert sorted(c['definition']['member'] for c in data['cases']) == ['gear','pinion']
    tolerance = f.point(data['value_tolerance_mm'])[0]
    assert tolerance > 0
    points = sweeps = 0
    cover_count = cell_count = certified_points = 0
    margins = []
    members = {}
    for case in data['cases']:
        d = case['definition']
        assert isinstance(d['teeth'],int) and d['teeth'] > 0
        assert len(d['indices']) == d['teeth'], 'incomplete tooth indexing'
        assert case['points'], 'no material witnesses'
        motion = d['motion_definition']
        domain = f.interval(d['roll_domain'])
        for row in case['points']:
            p = f.vec(row['position_mm'])
            blank = f.interval(row['blank'])
            actual_blank = f.field(p,d['blank'])
            encloses(blank,actual_blank,'blank')
            material = blank
            inside = actual_blank[1] < 0
            outside = actual_blank[0] > 0
            margin = -actual_blank[1]
            assert len(row['sweeps']) == len(d['indices']), 'missing indexed sweep'
            for index,sweep in zip(d['indices'],row['sweeps']):
                assert index['ratio'] == 0, 'index must be a fixed pose'
                assert sweep['status'] == 'Converged'
                t = sweep['roll_witness']
                assert domain[0] <= f.point(t)[0] <= domain[1], 'roll outside domain'
                minimum = f.interval(sweep['minimum'])
                assert minimum[1]-minimum[0] <= tolerance, 'minimum width'
                # inverse(M(t)) inverse(index) x; all coefficients are exact
                # binary64 inputs to the independent rational interval evaluator.
                indexed = f.rotation(p,index,0,inverse=True)
                q = f.rotation(indexed,motion['observer'],t)
                q = f.rotation(q,motion['source'],t,inverse=True)
                actual = f.field(q,d['generator'])
                encloses(minimum,actual,'indexed attained witness')
                outside |= actual[1] < 0
                if data['schema'] == 5 and row['expected_inside']:
                    n,lower = roll_cover.check(indexed,motion,d['generator'],domain,sweep['positive_cover'])
                    cell_count += n
                    cover_count += 1
                    margin = min(margin,lower)
                material = f.maximum(material,f.neg(minimum))
                sweeps += 1
            assert material == f.interval(row['material']), 'blank-minus-sweeps composition'
            if row['expected_inside']:
                assert material[1] < 0, 'unestablished material'
            else:
                assert material[0] > 0, 'unestablished exterior'
            if data['schema'] == 5:
                if row['expected_inside']:
                    assert inside and margin > 0, 'independent material proof failed'
                    margins.append(margin)
                else:
                    assert outside, 'independent exterior proof failed'
                certified_points += 1
            points += 1
            if progress:
                print(f"{d['member']} {row['label']}: point {points}, {cell_count} positive-cover cells",file=sys.stderr,flush=True)
        members[d['member']] = {'teeth':d['teeth'],'points':len(case['points'])}
    result = {'points':points,'indexed_sweep_witnesses':sweeps,'members':members,
        'arithmetic':'outward rational intervals',
        'scope':'blank bounds, attained sweep witnesses and interval CSG algebra; sweep lower bounds and source error not independently audited'}
    if data['schema'] == 5:
        result.update({'independently_classified_points':certified_points,
            'positive_roll_covers':cover_count,'positive_roll_cells':cell_count,
            'minimum_retained_field_margin_mm':float(min(margins)),
            'scope':'point material signs for explicit binary64 geometry, using complete roll covers; tight minimum intervals, source error and whole-space/export guarantees not certified'})
    return result


def negative_controls(data):
    rejected = []
    for change in ['blank','sweep','index','missing_index','cut_sign']:
        bad = deepcopy(data)
        case = bad['cases'][0]
        row = case['points'][0]
        if change == 'blank': row['blank'] = [100,101]
        elif change == 'sweep': row['sweeps'][0]['minimum'] = [-0.5001,-0.5]
        elif change == 'index': case['definition']['indices'][0]['phase'] += 0.1
        elif change == 'missing_index':
            case['definition']['indices'].pop()
            for r in case['points']: r['sweeps'].pop()
        else: row['material'] = [-row['material'][1],-row['material'][0]]
        try:
            check(bad)
        except AssertionError:
            rejected.append(change)
        else:
            raise AssertionError('invalid member evidence accepted: '+change)
    if data['schema'] == 5:
        for change in ['missing_cover_cell','false_positive_bound']:
            bad = deepcopy(data)
            case = bad['cases'][0]
            row = next(r for r in case['points'] if r['expected_inside'])
            sweep = next(s for s in row['sweeps'] if len(s['positive_cover']['intervals']) > 2)
            cells = sweep['positive_cover']['intervals']
            if change == 'missing_cover_cell': cells.pop(len(cells)//2)
            else: sweep['positive_cover']['intervals'] = [case['definition']['roll_domain']]
            try:
                check(bad)
            except AssertionError:
                rejected.append(change)
            else:
                raise AssertionError('invalid roll cover accepted: '+change)
    return rejected


if __name__ == '__main__':
    assert len(sys.argv) == 2, 'provide the complete-member JSON'
    roll_cover.self_checks()
    data = json.loads(Path(sys.argv[1]).read_text())
    result = check(data,progress=True)
    result['rejected_controls'] = negative_controls(data)
    print(json.dumps(result,indent=2))
