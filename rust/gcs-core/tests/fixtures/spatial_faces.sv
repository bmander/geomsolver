unit mm
use std
in std.front {
  o := point
  q := point
  x := point
  a := point
  b := point
  m := point
  c := point
  d := point
  fix(x == 0, y == 0) o
  fix(x == 0, y == 2) q
  fix(x == 1, y == 0) x
  fix(x == 2, y == 0) a
  fix(x == 3, y == 0) b
  fix(x == 3, y == 1) m
  fix(x == 3, y == 2) c
  fix(x == 2, y == 2) d
  ax := line(o,q)
  spin_axis := line(o,x)
  bottom := line(a,b)
  low := line(b,m)
  high := line(m,c)
  top := line(c,d)
  inner := line(d,a)
}
profile := face(bottom,low,high,top,inner)
body := solid(profile,about: ax)
first_surface := surface(body,low,from: 0deg,to: 90deg)
second_surface := surface(body,high,from: 0deg,to: 90deg)
roll := motion(about: spin_axis)
first_envelope := envelope(first_surface,under: roll,from: -20deg,to: 20deg)
second_envelope := envelope(second_surface,under: roll,from: -20deg,to: 20deg)
shared := seam(first_envelope,second_envelope)

component Sphere(origin: point,size: Length) {
  private south := point hint(x: origin.x,y: origin.y-size)
  private north := point hint(x: origin.x,y: origin.y+size)
  south vertical origin
  north vertical origin
  private rim := arc(center: origin,start: south,end: north)
  radius(size) rim
  private diameter := line(north,south)
  private carrier := solid(face(rim,diameter),about: diameter)
  wall := surface(carrier,rim)
}
in std.front {
  shifted := point
  fix(x == 0, y == 1) shifted
  globe := Sphere(o,size: sqrt(9.25)*1mm)
  offset := Sphere(shifted,size: sqrt(10.25-cos(0.1rad))*1mm)
  join_cut := Sphere(shifted,size: sqrt(11-2*cos(0.1rad))*1mm)
}
radial := seam(first_envelope,globe.wall)
offset_edge := seam(first_envelope,offset.wall)
join_edge := seam(second_envelope,join_cut.wall)
corner := vertex(first: radial,second: offset_edge)
junction := vertex(shared,join_edge)

in std.front {
  lower := Sphere(o,size: sqrt(9+0.98*0.98)*1mm)
  upper := Sphere(o,size: sqrt(9+1.02*1.02)*1mm)
  far_cut := Sphere(shifted,size: sqrt(11-2*cos(0.2rad))*1mm)
}
low_cut := seam(first_envelope,lower.wall)
high_cut := seam(second_envelope,upper.wall)
near_low := seam(first_envelope,join_cut.wall)
far_low := seam(first_envelope,far_cut.wall)
near_high := seam(second_envelope,join_cut.wall)
far_high := seam(second_envelope,far_cut.wall)
bl := vertex(low_cut,near_low)
br := vertex(low_cut,far_low)
ml := vertex(shared,near_low)
mr := vertex(shared,far_low)
tl := vertex(high_cut,near_high)
tr := vertex(high_cut,far_high)
bottom_edge := edge(low_cut,from: bl,to: br,along: ax)
middle_edge := edge(shared,from: ml,to: mr,along: ax)
top_edge := edge(high_cut,from: tl,to: tr,along: ax)
left_low := edge(near_low,from: bl,to: ml,along: ax)
right_low := edge(far_low,from: br,to: mr,along: ax)
left_high := edge(near_high,from: ml,to: tl,along: ax)
right_high := edge(far_high,from: mr,to: tr,along: ax)

// Spatial face declarations

construction lower_face := face(left_low,middle_edge,right_low,bottom_edge,on: first_envelope)
upper_face := face(left_high,top_edge,right_high,middle_edge,on: second_envelope)
