//! **`--png`: what the workspace shows, without a browser or a GPU** (#541).
//!
//! A part is checked by looking at it, and where agents work there is no page to look at.  So
//! the CLI draws what the box draws (`web/src/app/box3d.ts`): the objects the workspace shows
//! (`overview::shown_objects`), each the mesh the core cuts it to there
//! (`ApproximationPolicy::Mesh`, `solid::mesh_unit`), placed by its own placement, with the
//! creases the box strokes over it — and its silhouettes, which a picture without an orbit needs
//! to read a round thing as round.  Opened on a part file, that is its `preview`: the solids the
//! preview builds, about the world's origin, as the workspace opens the file.
//!
//! **A depth buffer, not a painter's sort.**  Each triangle is scan-converted at twice the
//! picture's resolution and kept per pixel where it is nearest the eye, then the edges are drawn
//! where nothing stands nearer than they do, and the whole is averaged down.  A centroid sort
//! draws a small face behind a big one through it; this cannot.
//!
//! **Nothing here is geometry the core has not already given.**  The eye is the core's
//! (`workspace::eye_frame`, the box's camera), the triangles, normals and edges are the evaluated
//! solid's, and what is left — projecting, filling, shading and an encoder — is a renderer's.

use gcs_core::model::Sketch;
use gcs_core::overview::workspace::eye_frame;
use gcs_core::solid::{ApproximationPolicy, LocalPoint};

/// The eye a picture is taken from, as the box's orbit holds it: a bearing and an elevation.
#[derive(Clone, Copy, Debug)]
pub struct Eye {
    pub az: f64,
    pub el: f64,
}

/// The eye a document with solids opens with (`view.ts`'s `THREE_QUARTER`): what `--png` draws
/// unless told otherwise.
pub const HOME: Eye = Eye { az: -std::f64::consts::PI / 3.0, el: std::f64::consts::PI / 6.0 };

/// The eye `--view` names: one of the standard planes' squarely (front from −y, side from +x,
/// top from +z, as `std.sv` stands them), `iso` the isometric from the front's right, `home` the
/// workspace's opening view, or a bearing and an elevation in degrees, `az,el`.
pub fn eye(name: &str) -> Result<Eye, String> {
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};
    Ok(match name {
        "home" => HOME,
        "front" => Eye { az: -FRAC_PI_2, el: 0.0 },
        "side" => Eye { az: 0.0, el: 0.0 },
        "top" => Eye { az: -FRAC_PI_2, el: FRAC_PI_2 },
        "iso" => Eye { az: -FRAC_PI_4, el: (1.0 / 3f64.sqrt()).asin() },
        _ => {
            let angles: Vec<f64> = name.split(',').filter_map(|v| v.trim().parse().ok()).collect();
            match angles[..] {
                [az, el] if name.split(',').count() == 2 && az.is_finite() && el.is_finite() =>
                    Eye { az: az.to_radians(), el: el.clamp(-90.0, 90.0).to_radians() },
                _ => return Err(format!("--view needs iso, front, side, top, home or az,el in degrees, not `{name}`")),
            }
        }
    })
}

/// What `--png` was asked for.
pub struct Ask<'a> {
    pub eye: Eye,
    /// The picture's longer side, in pixels.
    pub size: usize,
    /// `--solid`: one solid, or the objects of one instance, rather than every object shown.
    pub solid: Option<&'a str>,
    /// `--hide`: the objects to leave out, each by a name and everything under it (a cutaway).
    pub hide: &'a [String],
}

/// Whether `name` is `asked` or something under it: `head` reaches `head.casting`.
fn under(name: &str, asked: &str) -> bool {
    name == asked || name.strip_prefix(asked).is_some_and(|r| r.starts_with('.'))
}

/// The solids a picture draws: the one `--solid` names (else the objects under that name), or
/// every object the workspace shows; less those `--hide` names.
pub fn chosen(sk: &Sketch, ask: &Ask) -> Result<Vec<usize>, String> {
    let named = |i: usize| sk.solids[i].name.as_str();
    let mut v = match ask.solid {
        Some(n) => match sk.solids.iter().position(|s| s.name == n) {
            Some(i) => vec![i],
            None => {
                let v: Vec<usize> = gcs_core::overview::objects(sk).into_iter()
                    .filter(|&i| under(named(i), n)).collect();
                if v.is_empty() { return Err(format!("no solid called `{n}`")); }
                v
            }
        },
        None => gcs_core::overview::shown_objects(sk),
    };
    for h in ask.hide {
        if !sk.solids.iter().any(|s| under(&s.name, h)) {
            return Err(format!("no solid called `{h}` to hide"));
        }
    }
    v.retain(|&i| !ask.hide.iter().any(|h| under(named(i), h)));
    if v.is_empty() { return Err("this document has no solid to draw".into()); }
    Ok(v)
}

/// The paper behind, the ink of an edge, and the colours an instance is drawn in: the box's
/// background and edge (`box3d.ts`'s `INK`), and pigments light enough to shade.
const PAPER: [f64; 3] = [250.0, 250.0, 250.0];
const EDGE: [f64; 3] = [58.0, 63.0, 69.0];
const PIGMENTS: [[f64; 3]; 10] = [
    [143.0, 179.0, 217.0], [224.0, 163.0, 112.0], [146.0, 196.0, 143.0], [217.0, 140.0, 140.0],
    [181.0, 159.0, 214.0], [196.0, 164.0, 132.0], [227.0, 166.0, 207.0], [176.0, 179.0, 184.0],
    [209.0, 207.0, 122.0], [127.0, 200.0, 207.0],
];

/// A solid's instance: the first step of its name.
fn instance(name: &str) -> &str {
    name.split('.').next().unwrap_or(name)
}

/// The document's instances, in the order its objects come in: all of them, so a part keeps its
/// colour in a cutaway or alone.
fn instances(sk: &Sketch) -> Vec<&str> {
    let mut seen = Vec::new();
    for o in gcs_core::overview::objects(sk) {
        let name = instance(&sk.solids[o].name);
        if !seen.contains(&name) { seen.push(name); }
    }
    seen
}

/// A solid's colour: its instance's, so every solid of one part is one colour and the first ten
/// parts are told apart.  A solid of no object's instance (an operand drawn by `--solid`) is
/// coloured by its name's hash.
fn pigment(instances: &[&str], name: &str) -> [f64; 3] {
    let own = instance(name);
    let k = instances.iter().position(|n| *n == own).unwrap_or_else(|| {
        own.bytes().fold(0x811c_9dc5u32, |h, b| (h ^ b as u32).wrapping_mul(0x0100_0193)) as usize
    });
    PIGMENTS[k % PIGMENTS.len()]
}

/// How many samples a side each pixel is averaged from.
const SAMPLES: usize = 2;
/// The margin round the drawing, as a fraction of the picture's longer side.
const MARGIN: f64 = 0.04;
/// The light: mostly from over the eye's left shoulder, and how much falls everywhere.
const AMBIENT: f64 = 0.38;
const LIGHT: [f64; 3] = [-0.35, 0.55, 0.76];

/// One solid seen: its triangles and normals in the world, and the edges to draw.
struct Seen {
    pigment: [f64; 3],
    positions: Vec<[f64; 3]>,
    normals: Vec<[f64; 3]>,
    edges: Vec<[[f64; 3]; 2]>,
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// The PNG of `which` seen from `ask.eye`, and each solid that did not evaluate with the core's
/// reason (left out of the picture, as the box leaves it out).
pub fn render(sk: &Sketch, which: &[usize], ask: &Ask) -> Result<(Vec<u8>, Vec<String>), String> {
    let [right, up, toward] = eye_frame(ask.eye.az, ask.eye.el);
    // the eye's coordinates of a world point: across, up, and toward the viewer
    let see = |p: [f64; 3]| [dot(right, p), dot(up, p), dot(toward, p)];
    let instances = instances(sk);
    let mut seen = Vec::new();
    let mut unbuilt = Vec::new();
    for &i in which {
        let solid = match sk.evaluated_solid(i, ApproximationPolicy::Mesh) {
            Ok(s) => s,
            Err(why) => { unbuilt.push(why); continue; }
        };
        let m = solid.mesh();
        let world = |p: &[f64; 3]| solid.to_world(LocalPoint(*p)).0;
        let normal = |n: &[f64; 3]| solid.vector_to_world(*n);
        // the creases the box strokes, and the silhouettes as the core's renderer draws them
        // (`renderer::projection`): on an exact solid traced on its surfaces, else each seam of
        // a round face where its facets turn from facing the eye to facing away
        let traced = solid.silhouettes(solid.vector_to_local(toward));
        let seam = |e: &gcs_core::csg::Edge| {
            let (a, b) = (dot(normal(&e.na), toward), dot(normal(&e.nb), toward));
            traced.is_none() && a * b <= 0.0 && (a.abs() >= 1e-12 || b.abs() >= 1e-12)
        };
        let edges = solid.edges().iter().filter(|e| !e.smooth || seam(e))
            .chain(traced.iter().flatten())
            .map(|e| [see(world(&e.a)), see(world(&e.b))])
            .collect();
        seen.push(Seen {
            pigment: pigment(&instances, &sk.solids[i].name),
            positions: m.positions.as_chunks::<3>().0.iter().map(|p| see(world(p))).collect(),
            normals: m.normals.as_chunks::<3>().0.iter().map(|n| see(normal(n))).collect(),
            edges,
        });
    }
    if seen.iter().all(|s| s.positions.is_empty()) {
        return Err(unbuilt.first().cloned().unwrap_or_else(|| "this document has no solid to draw".into()));
    }
    // the picture fitted to what it draws, its longer side `size` and its shorter at least half
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for p in seen.iter().flat_map(|s| &s.positions) {
        for k in 0..2 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    let span = [(hi[0] - lo[0]).max(1e-12), (hi[1] - lo[1]).max(1e-12)];
    let size = ask.size as f64;
    let aspect = (span[1] / span[0]).clamp(0.5, 2.0);
    let (w, h) = if aspect <= 1.0 { (size, (size * aspect).round()) } else { ((size / aspect).round(), size) };
    let (w, h) = (w.max(1.0) as usize, h.max(1.0) as usize);
    let margin = 2.0 * MARGIN * size;
    let scale = ((w as f64 - margin) / span[0]).min((h as f64 - margin) / span[1]) * SAMPLES as f64;
    let (cx, cy) = ((lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0);
    let mut canvas = Canvas::new(w * SAMPLES, h * SAMPLES);
    // a point in the eye's coordinates on the canvas, its depth kept
    let (half_w, half_h) = (canvas.w as f64 / 2.0, canvas.h as f64 / 2.0);
    let place = |p: [f64; 3]| [(p[0] - cx) * scale + half_w, half_h - (p[1] - cy) * scale, p[2]];
    let light = {
        let l = LIGHT;
        let n = dot(l, l).sqrt();
        l.map(|x| x / n)
    };
    for s in &seen {
        for (t, n) in s.positions.as_chunks::<3>().0.iter().zip(s.normals.as_chunks::<3>().0) {
            canvas.triangle(t.map(place), *n, s.pigment, light);
        }
    }
    // an edge is drawn where it is no farther than the surface it bounds, give or take what a
    // sample's width of a steep face spans in depth
    let depth = seen.iter().flat_map(|s| &s.positions).fold((f64::INFINITY, f64::NEG_INFINITY),
        |(a, b), p| (a.min(p[2]), b.max(p[2])));
    let slack = (depth.1 - depth.0) * 2e-3 + 2.5 / scale;
    for s in &seen {
        for e in &s.edges {
            canvas.line(place(e[0]), place(e[1]), slack);
        }
    }
    let rgb = canvas.down(SAMPLES);
    Ok((crate::png::encode(w, h, &rgb), unbuilt))
}

/// The samples being drawn: a colour and a depth each, the depth `-∞` where nothing is.
struct Canvas {
    w: usize,
    h: usize,
    colour: Vec<[f64; 3]>,
    depth: Vec<f64>,
}

impl Canvas {
    fn new(w: usize, h: usize) -> Canvas {
        Canvas { w, h, colour: vec![PAPER; w * h], depth: vec![f64::NEG_INFINITY; w * h] }
    }

    /// One triangle, its corners on the canvas with their depths and its normals the eye's, each
    /// sample it covers kept where it is the nearest yet: shaded by the normal it interpolates,
    /// so a round face reads round and a flat one (whose three normals agree) flat.  Either side
    /// is lit alike, a sheet's back as its front.
    fn triangle(&mut self, p: [[f64; 3]; 3], n: [[f64; 3]; 3], pigment: [f64; 3], light: [f64; 3]) {
        let area = (p[1][0] - p[0][0]) * (p[2][1] - p[0][1]) - (p[2][0] - p[0][0]) * (p[1][1] - p[0][1]);
        if !area.is_finite() || area.abs() < 1e-12 { return; }
        let x0 = p.iter().map(|q| q[0]).fold(f64::INFINITY, f64::min).floor().max(0.0) as usize;
        let x1 = (p.iter().map(|q| q[0]).fold(f64::NEG_INFINITY, f64::max).ceil() as isize).min(self.w as isize - 1);
        let y0 = p.iter().map(|q| q[1]).fold(f64::INFINITY, f64::min).floor().max(0.0) as usize;
        let y1 = (p.iter().map(|q| q[1]).fold(f64::NEG_INFINITY, f64::max).ceil() as isize).min(self.h as isize - 1);
        if x1 < 0 || y1 < 0 { return; }
        // a sample is covered when it is on the inside of all three edges; one exactly on an edge
        // is the triangle's whose side the area's sign puts it, so neighbours leave no crack
        let edge = |a: [f64; 3], b: [f64; 3], x: f64, y: f64| (b[0] - a[0]) * (y - a[1]) - (b[1] - a[1]) * (x - a[0]);
        for y in y0..=y1 as usize {
            let sy = y as f64 + 0.5;
            for x in x0..=x1 as usize {
                let sx = x as f64 + 0.5;
                let w0 = edge(p[1], p[2], sx, sy) / area;
                let w1 = edge(p[2], p[0], sx, sy) / area;
                let w2 = 1.0 - w0 - w1;
                if w0 < 0.0 || w1 < 0.0 || w2 < 0.0 { continue; }
                let z = w0 * p[0][2] + w1 * p[1][2] + w2 * p[2][2];
                let k = y * self.w + x;
                if z <= self.depth[k] { continue; }
                self.depth[k] = z;
                let m: [f64; 3] = std::array::from_fn(|c| w0 * n[0][c] + w1 * n[1][c] + w2 * n[2][c]);
                let len = dot(m, m).sqrt();
                let lit = if len > 0.0 { (dot(m, light) / len).abs() } else { 1.0 };
                let shade = AMBIENT + (1.0 - AMBIENT) * lit;
                self.colour[k] = pigment.map(|c| c * shade);
            }
        }
    }

    /// An edge, three samples wide, inked wherever nothing stands nearer than it by more than
    /// `slack`.  At a silhouette only the outer side survives the test, the surface turning
    /// toward the eye just inside it, so a stroke any thinner would be half a pixel there.
    fn line(&mut self, a: [f64; 3], b: [f64; 3], slack: f64) {
        let steps = (b[0] - a[0]).abs().max((b[1] - a[1]).abs()).ceil().max(1.0);
        if !steps.is_finite() { return; }
        for s in 0..=steps as usize {
            let t = s as f64 / steps;
            let q: [f64; 3] = std::array::from_fn(|c| a[c] + t * (b[c] - a[c]));
            let (cx, cy) = ((q[0] - 0.5).round() as isize, (q[1] - 0.5).round() as isize);
            for (dx, dy) in (-1..=1).flat_map(|dx| (-1..=1).map(move |dy| (dx, dy))) {
                let (x, y) = (cx + dx, cy + dy);
                if x < 0 || y < 0 || x >= self.w as isize || y >= self.h as isize { continue; }
                let k = y as usize * self.w + x as usize;
                if q[2] + slack >= self.depth[k] {
                    self.colour[k] = EDGE;
                }
            }
        }
    }

    /// The picture: each `n`-by-`n` block of samples averaged into a pixel, three bytes each.
    fn down(&self, n: usize) -> Vec<u8> {
        let (w, h) = (self.w / n, self.h / n);
        let mut out = Vec::with_capacity(w * h * 3);
        for y in 0..h {
            for x in 0..w {
                let mut sum = [0.0; 3];
                for j in 0..n {
                    for i in 0..n {
                        let c = self.colour[(y * n + j) * self.w + x * n + i];
                        for k in 0..3 { sum[k] += c[k]; }
                    }
                }
                out.extend(sum.map(|v| (v / (n * n) as f64).round().clamp(0.0, 255.0) as u8));
            }
        }
        out
    }
}
