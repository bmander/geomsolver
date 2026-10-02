unit mm
fp := point hint(x: 0, y: 0)
ground fp
fc := circle(center: fp) hint(r: 1e-06)
radius(1e-06mm) fc
f := face(fc)
result := solid(f, depth: 5mm)
