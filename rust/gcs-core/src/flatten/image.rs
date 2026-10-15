//! **A face carried whole** (#357, Solvent §6.8 [0.116]): `sec := prof mapped(from -> a, to ->
//! b) in n` is the face `prof`, its loop walked as a block walks it (`resolve::face_walk`), each
//! corner carried by the map as a point is (`q mapped(…) p`, `CKind::Mapped`) and each edge
//! rebuilt between the images in the plane `n`.  Built here, where the loop can be walked, as the
//! statements a person would write for it, walked in the scope the definition stands in:
//!
//! * a **corner** of a face of corners keeps its name, `sec.k0`, and the image is `face(sec.k0,
//!   …, -> close)`, whose closing runs name its walls as the source's do (`close0`, …);
//! * an **edge** keeps its name, `sec.ab`, between corners keyed under the image (`sec#q0`, the
//!   image of the edge's start, `p1` of a line and `start` of an arc — the walk's own reading,
//!   `std.Affine`'s): a line a line, and under the similarity of two pairs an arc an arc about
//!   its centre's image and a circle a circle (`CKind::MappedCircle`), its radius scaled;
//! * **the mirror** (`mirror: true`) reverses the turn, so the image is walked the other way and
//!   an arc runs from its end's image to its start's: the image winds as the source does.
//!
//! Under an affine map of three pairs a circle is an ellipse, which a circle cannot be: an arc or
//! a circle there is refused, naming it.  So are a spline and a curve, which no map carries yet.

use super::*;
use crate::model::EntKind;
use crate::program::public_path;
use crate::syntax::{
    Arg, Chained, DeclName, FaceImage, Membership, OpArg, Relation, RelationForm, Written,
};

/// One link of the loop the image is built over: what it is, read where it was written.
enum Part {
    /// A corner of a face of corners: its name, read in the face's scope.
    Corner { at: Ref, scope: Scope, name: String },
    /// An edge: its kind and name, read in the scope the loop names it in.
    Edge { at: Ref, scope: Scope, kind: EntKind, name: String },
}

impl Walk<'_> {
    /// The image of `p`'s face over its loop (`links`), in the scope the definition stands in,
    /// spliced in where the definition stood in the walk's output (`at`).
    pub(super) fn image_face(
        &mut self,
        p: &Pending,
        img: &FaceImage,
        links: &[Link],
        at_out: usize,
    ) {
        let alias = self.alias_table();
        let sec = img.name.text.clone();
        let at = img.word.span;
        let pairs = img.args.iter().filter(|a| matches!(a, OpArg::Ent(_))).count() / 2;
        let mirror = img.args.iter().any(|a| matches!(a, OpArg::Named(n, Arg::Bool(true))
            if n.text == "mirror"));
        let mut parts = Vec::with_capacity(links.len());
        for (k, link) in links.iter().enumerate() {
            match link {
                // a corner by its path as written (`k0`, `x0.p`)
                Link::Run { from, scope, .. } => {
                    let name = ref_text(from);
                    let plain = name.split('.').all(crate::syntax::is_name);
                    let name = if plain { name } else { format!("#k{k}") };
                    parts.push(Part::Corner { at: from.clone(), scope: scope.clone(), name });
                }
                Link::Edge(r, sc) => {
                    let Some((abs, _)) = self.look(&alias).find(r, sc) else {
                        let (code, m) = self.look(&alias).missing(r, sc);
                        self.err(code, r.span, m);
                        return;
                    };
                    let kind = self.out.iter().find_map(|(st, _, _)| match &st.kind {
                        StmtKind::Decl(d) if d.name.key().text == abs => Some(d.kind),
                        _ => None,
                    });
                    let public = public_path(&abs);
                    let leaf = public.rsplit('.').next().unwrap_or(&public).to_string();
                    let name = if crate::syntax::is_name(&leaf) { leaf } else { format!("#e{k}") };
                    let Some(kind) = kind else {
                        self.err(Code::E080, at, format!("`{public}` is no edge a map carries"));
                        return;
                    };
                    parts.push(Part::Edge { at: r.clone(), scope: sc.clone(), kind, name });
                }
            }
        }
        // what the map cannot carry, said naming the edge
        for part in &parts {
            let Part::Edge { kind, name, .. } = part else { continue };
            let why = match kind {
                EntKind::Line => continue,
                EntKind::Arc | EntKind::Circle if pairs == 2 => continue,
                EntKind::Arc | EntKind::Circle => format!(
                    "`{name}` is {}, and under a map of three pairs a circle is an ellipse, \
                     which {} cannot be: carry it by two pairs, the similarity",
                    kind.a(), kind.a()
                ),
                _ => format!("`{name}` is {}, which a map does not carry yet", kind.a()),
            };
            self.err(Code::E080, at, why);
            return;
        }
        if parts.iter().any(|p| matches!(p, Part::Edge { kind: EntKind::Circle, .. }))
            && parts.len() != 1
        {
            self.err(Code::E080, at, "a circle is a loop of its own, and no edge of a longer one");
            return;
        }
        // a name the image gives twice, or one the source never wrote, is a key no source writes
        let names: Vec<String> = parts.iter().map(|p| match p {
            Part::Corner { name, .. } | Part::Edge { name, .. } => name.clone(),
        }).collect();
        let named = |k: usize| -> DeclName {
            let text = &names[k];
            let once = names.iter().filter(|n| *n == text).count() == 1;
            match once && !text.starts_with('#') {
                true => DeclName::Written(Name { text: format!("{sec}.{text}"), span: at }),
                false => DeclName::Key(Name { text: format!("{sec}#{k}"), span: at }),
            }
        };
        // each link's source, an alias the statements below read it by
        let prefix = p.scope.prefix().to_string();
        let src = |k: usize| format!("{sec}#src{k}");
        for (k, part) in parts.iter().enumerate() {
            let (r, sc) = match part {
                Part::Corner { at, scope, .. } | Part::Edge { at, scope, .. } => (at, scope),
            };
            self.aliases.push((format!("{prefix}{}", src(k)), r.clone(), sc.clone()));
        }
        let mut body = Vec::new();
        let stmt =
            |kind: StmtKind| Stmt { id: p.st.id, kind, span: p.st.span, chained: Chained::No };
        // a point of the image, drawn in `n`, seeded where its source is seen there: apart from
        // the others, so an arc's image starts with a radius (at none, its rows have no slope)
        let point = |name: DeclName, of: &Ref| Decl {
            membership: Membership::written_at(img.plane.clone(), Span::default()),
            ..Decl::point(name, [0.0; 2], Some(crate::syntax::AtRef::at(of.clone())))
        };
        let carried = |what: Ref, of: Ref| Relation {
            form: RelationForm::Written(Written {
                word: img.word.clone(),
                fixity: crate::constraints::Fixity::Infix,
                ops: vec![what, of],
                args: img.args.clone(),
                span: at,
            }),
            place: None,
            place_span: Span::default(),
            claim: false,
            class: Default::default(),
            class_span: Span::default(),
            word: None,
            along: None,
            space: false,
        };
        let local = |text: &str| Ref::dotted(text, at);
        let n = parts.len();
        let mut loop_kids: Vec<Kid> = Vec::with_capacity(n);
        let face_of_corners = parts.iter().all(|p| matches!(p, Part::Corner { .. }));
        if face_of_corners {
            for k in 0..n {
                let name = named(k);
                let q = name.key().text.clone();
                let of = local(&src(k));
                let decl = point(name, &of);
                body.push(stmt(StmtKind::Decl(decl)));
                body.push(stmt(StmtKind::Relation(carried(local(&q), of))));
                loop_kids.push(Kid::Ref(local(&q)));
            }
        } else if let [Part::Edge { kind: EntKind::Circle, .. }] = parts.as_slice() {
            // a circle about its centre's image, its radius scaled (`CKind::MappedCircle`)
            let name = named(0);
            let k = name.key().text.clone();
            let c = format!("{sec}#c0");
            let of = local(&format!("{}.center", src(0)));
            body.push(stmt(StmtKind::Decl(point(DeclName::Key(Name { text: c.clone(), span: at }),
                                                &of))));
            let circle = Decl {
                kind: EntKind::Circle,
                children: vec![vec![Kid::Ref(local(&c))]],
                // no seed: the radius is read off the centre and an end once they are placed
                seed: Vec::new(),
                seed_text: Vec::new(),
                membership: Membership::written_at(img.plane.clone(), Span::default()),
                ..Decl::point(name, [0.0; 2], None)
            };
            body.push(stmt(StmtKind::Decl(circle)));
            body.push(stmt(StmtKind::Relation(carried(local(&k), local(&src(0))))));
            loop_kids.push(Kid::Ref(local(&k)));
        } else {
            // the image of each edge's start, the corners the images meet at
            let q = |k: usize| format!("{sec}#q{}", k % n);
            for (k, part) in parts.iter().enumerate() {
                let Part::Edge { kind, .. } = part else {
                    self.err(Code::E080, at, "a face is carried by its corners or by its edges");
                    return;
                };
                let start = if *kind == EntKind::Arc { "start" } else { "p1" };
                let of = local(&format!("{}.{start}", src(k)));
                let decl = point(DeclName::Key(Name { text: q(k), span: at }), &of);
                body.push(stmt(StmtKind::Decl(decl)));
                body.push(stmt(StmtKind::Relation(carried(local(&q(k)), of))));
            }
            for (k, part) in parts.iter().enumerate() {
                let Part::Edge { kind, .. } = part else { unreachable!("edges") };
                let name = named(k);
                let e = name.key().text.clone();
                let decl = match kind {
                    EntKind::Arc => {
                        let c = format!("{sec}#c{k}");
                        let of = local(&format!("{}.center", src(k)));
                        let centre = point(DeclName::Key(Name { text: c.clone(), span: at }), &of);
                        body.push(stmt(StmtKind::Decl(centre)));
                        body.push(stmt(StmtKind::Relation(carried(local(&c), of))));
                        // the mirror reverses the turn: the arc runs from its end's image
                        let (s, t) = if mirror { (q(k + 1), q(k)) } else { (q(k), q(k + 1)) };
                        Decl {
                            kind: EntKind::Arc,
                            children: vec![vec![Kid::Ref(local(&c))], vec![Kid::Ref(local(&s))],
                                           vec![Kid::Ref(local(&t))]],
                            seed: Vec::new(),
                            seed_text: Vec::new(),
                            membership: Membership::written_at(img.plane.clone(), Span::default()),
                            ..Decl::point(name, [0.0; 2], None)
                        }
                    }
                    _ => Decl {
                        membership: Membership::written_at(img.plane.clone(), Span::default()),
                        ..Decl::line(name, local(&q(k)), local(&q(k + 1)), Default::default())
                    },
                };
                body.push(stmt(StmtKind::Decl(decl)));
                loop_kids.push(Kid::Ref(local(&e)));
            }
        }
        // the mirror reverses the turn, so the image is walked the other way round
        if mirror {
            loop_kids.reverse();
        }
        let face = Decl {
            kind: EntKind::Face,
            children: vec![loop_kids, Vec::new(), Vec::new()],
            seed: Vec::new(),
            seed_text: Vec::new(),
            close: face_of_corners.then_some(at),
            ..Decl::point(DeclName::Written(img.name.clone()), [0.0; 2], None)
        };
        body.push(stmt(StmtKind::Decl(face)));
        let mut vals = p.vals.clone();
        let outer = std::mem::take(&mut self.out);
        self.aside.held = outer.len();
        self.body(&body, &p.scope, &mut vals, &p.path, p.depth + 1);
        let made = std::mem::replace(&mut self.out, outer);
        self.aside.held = 0;
        self.out.splice(at_out..at_out, made);
    }
}
