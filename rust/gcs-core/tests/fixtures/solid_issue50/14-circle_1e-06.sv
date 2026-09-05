unit mm
point fp hint(x: 0, y: 0)
ground fp
circle fc(center: fp) hint(r: 1e-06)
radius(1e-06mm) fc
face f(fc)
solid result(f, depth: 5mm)
