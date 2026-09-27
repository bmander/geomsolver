unit mm
point o hint(x: 0,y: 0)
point q hint(x: 0,y: 2)
point x hint(x: 1,y: 0)
point a hint(x: 2,y: 0)
point b hint(x: 3,y: 0)
point m hint(x: 3,y: 1)
point c hint(x: 3,y: 2)
point d hint(x: 2,y: 2)
ground o
ground q
ground x
ground a
ground b
ground m
ground c
ground d
line axis(o,q)
line spin_axis(o,x)
line bottom(a,b)
line low(b,m)
line high(m,c)
line top(c,d)
line inner(d,a)
face profile(bottom,low,high,top,inner)
solid body(profile,about: axis)
surface first_surface(body,low,from: 0deg,to: 90deg)
surface second_surface(body,high,from: 0deg,to: 90deg)
motion roll(about: spin_axis)
envelope first_envelope(first_surface,under: roll,from: -20deg,to: 20deg)
envelope second_envelope(second_surface,under: roll,from: -20deg,to: 20deg)
seam shared(first_envelope,second_envelope)

component Sphere(origin: point,size: Length) {
  private point south hint(x: origin.x,y: origin.y-size)
  private point north hint(x: origin.x,y: origin.y+size)
  ground south
  ground north
  private arc rim(center: origin,start: south,end: north)
  radius(size) rim
  private line diameter(north,south)
  private solid carrier(face(rim,diameter),about: diameter)
  surface wall(carrier,rim)
}
point shifted hint(x: 0,y: 1)
ground shifted
globe: Sphere(o,size: sqrt(9.25)*1mm)
offset: Sphere(shifted,size: sqrt(10.25-cos(0.1rad))*1mm)
join_cut: Sphere(shifted,size: sqrt(11-2*cos(0.1rad))*1mm)
seam radial(first_envelope,globe.wall)
seam offset_edge(first_envelope,offset.wall)
seam join_edge(second_envelope,join_cut.wall)
vertex corner(first: radial,second: offset_edge)
vertex junction(shared,join_edge)

lower: Sphere(o,size: sqrt(9+0.98*0.98)*1mm)
upper: Sphere(o,size: sqrt(9+1.02*1.02)*1mm)
far_cut: Sphere(shifted,size: sqrt(11-2*cos(0.2rad))*1mm)
seam low_cut(first_envelope,lower.wall)
seam high_cut(second_envelope,upper.wall)
seam near_low(first_envelope,join_cut.wall)
seam far_low(first_envelope,far_cut.wall)
seam near_high(second_envelope,join_cut.wall)
seam far_high(second_envelope,far_cut.wall)
vertex bl(low_cut,near_low)
vertex br(low_cut,far_low)
vertex ml(shared,near_low)
vertex mr(shared,far_low)
vertex tl(high_cut,near_high)
vertex tr(high_cut,far_high)
edge bottom_edge(low_cut,from: bl,to: br,along: axis)
edge middle_edge(shared,from: ml,to: mr,along: axis)
edge top_edge(high_cut,from: tl,to: tr,along: axis)
edge left_low(near_low,from: bl,to: ml,along: axis)
edge right_low(far_low,from: br,to: mr,along: axis)
edge left_high(near_high,from: ml,to: tl,along: axis)
edge right_high(far_high,from: mr,to: tr,along: axis)

// Spatial face declarations

construction face lower_face(left_low,middle_edge,right_low,bottom_edge,on: first_envelope)
face upper_face(left_high,top_edge,right_high,middle_edge,on: second_envelope)
