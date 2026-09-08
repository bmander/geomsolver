#!/usr/bin/env python3
"""Check Rust interval results using exact rational arithmetic, without loading Rust.

Input is the hexadecimal binary64 TSV written by SOLVENT_INTERVAL_OUTPUT in the
interval tests. Trigonometric references use a different, higher-degree exact
Taylor polynomial with its own rational remainder bound.
"""
from fractions import Fraction as F
from math import factorial
from pathlib import Path
import json
import struct
import sys


def rational(bits):
    value = struct.unpack('>d', bytes.fromhex(bits))[0]
    return F.from_float(value)


def reference_trig(x, cosine):
    term = F(1) if cosine else x
    total = term
    for k in range(1, 41):
        denominator = (2*k-1)*(2*k) if cosine else (2*k)*(2*k+1)
        term *= -x*x/denominator
        total += term
    degree = 80 if cosine else 81
    remainder = abs(x)**(degree+1)/factorial(degree+1)
    return total-remainder, total+remainder


def check(path):
    counts = {}
    for index, row in enumerate(Path(path).read_text().splitlines(), 1):
        op, *data = row.split('\t')
        a, b, c, d, lo, hi = map(rational, data)
        assert a <= b and c <= d and lo <= hi, (index, row)
        if op == 'add':
            expected = a+c, b+d
        elif op == 'sub':
            expected = a-d, b-c
        elif op == 'mul':
            values = [x*y for x in (a, b) for y in (c, d)]
            expected = min(values), max(values)
        elif op == 'div':
            assert not c <= 0 <= d
            values = [x/y for x in (a, b) for y in (c, d)]
            expected = min(values), max(values)
        elif op == 'square':
            expected = (F(0) if a <= 0 <= b else min(a*a, b*b)), max(a*a, b*b)
        elif op == 'sqrt':
            assert 0 <= lo and lo*lo <= a and hi*hi >= b, (index, row)
            expected = lo, hi
        elif op in ('sin', 'cos'):
            assert a == b and abs(a) <= 8
            expected = reference_trig(a, op == 'cos')
        else:
            raise AssertionError(f'unknown operation {op}')
        assert lo <= expected[0] <= expected[1] <= hi, (index, row, expected)
        counts[op] = counts.get(op, 0)+1
    assert set(counts) == {'add', 'sub', 'mul', 'div', 'square', 'sqrt', 'sin', 'cos'}
    return {'file': str(path), 'checked': sum(counts.values()), 'operations': counts}


if __name__ == '__main__':
    assert len(sys.argv) == 2, 'provide the exported arithmetic TSV'
    print(json.dumps(check(sys.argv[1]), indent=2))
