//! Compile named rigid motions with forward references and explicit dependency diagnostics.
use super::{resolve::{follow_building,Resolver},Code,Diag,Made,SourceMap};
use crate::{ir::{Decl,Operation,Statement},model::{EntKind,EntRef,MotionDef,MotionE,Sketch},
    measure::{Measured,MotionMeasure,MotionSlot},syntax::{Arg,MotionSpec,Ref,StmtId},units::Dim};
use std::collections::{BTreeMap,BTreeSet};

struct Builder<'a> {
    sk: &'a mut Sketch,
    res: &'a mut Resolver,
    map: &'a mut SourceMap,
    decls: BTreeMap<String,(&'a Statement,&'a Decl)>,
    done: BTreeMap<String,Result<usize,String>>,
    visiting: BTreeSet<String>,
    heights: Vec<usize>,
}

impl Builder<'_> {
    fn dependency(&mut self, r: &Ref) -> Result<usize,String> {
        if !r.path.is_empty() || !self.decls.contains_key(&r.root.text) {
            return Err(format!("`{}` does not name a motion",crate::syntax::ref_text(r)));
        }
        self.build(&r.root.text)
    }

    /// A number written as a measurement of the drawing (`length(a) / length(b)`): compiled
    /// against the entities its names denote, and worked out whenever the motion is read.
    fn measured(&self, a: &Option<Arg>, slot: MotionSlot) -> Result<Option<MotionMeasure>,String> {
        let Some(Arg::Dim {text,..}) = a else { return Ok(None); };
        if crate::expr::measurement_in(text,self.sk.units).is_none() { return Ok(None); }
        let (sk,res) = (&*self.sk,&*self.res);
        let value = Measured::compile(text,sk.units,slot.dim(),slot.label(),|name| {
            // the flattener wrote the name absolute: the longest head the resolver knows is the
            // declaration, and what follows it is a path into what that declaration made
            let segs: Vec<&str> = name.split('.').collect();
            let k = (1..=segs.len()).rev().find(|&k| res.of.contains_key(&segs[..k].join(".")))
                .ok_or_else(|| format!("no such entity: `{name}`"))?;
            let r = Ref {root:crate::syntax::Name::new(&segs[..k].join(".")),
                path:segs[k..].iter().map(|f| crate::syntax::Seg::Field(crate::syntax::Name::new(*f))).collect(),
                span:Default::default()};
            let e = res.lookup(&r).ok_or_else(|| format!("no such entity: `{name}`"))?;
            follow_building(sk,res,e,&r)
        })?;
        Ok(Some(MotionMeasure {slot,value}))
    }

    fn number(&self, a: &Option<Arg>, default: f64, dim: Dim, label: &str) -> Result<f64,String> {
        let Some(a) = a else { return Ok(default); };
        let Arg::Dim {text,..} = a else { return Err(format!("motion `{label}` needs a number")); };
        // measured, it has no number until the motion is read (`MotionE::rotation`); what stands
        // here is never read, and NaN says so to anything that would
        if crate::expr::measurement_in(text,self.sk.units).is_some() { return Ok(f64::NAN); }
        let a = crate::flatten::value_aff(text,&BTreeMap::new(),self.sk.units)?;
        a.dim.require(dim,label)?;
        let value = a.number().ok_or_else(|| format!("motion `{label}` must be bound"))?;
        if !value.is_finite() { return Err(format!("motion `{label}` must be finite")); }
        Ok(value)
    }

    fn build(&mut self, name: &str) -> Result<usize,String> {
        if let Some(done) = self.done.get(name) { return done.clone(); }
        if self.visiting.contains(name) {
            return Err(format!("motion dependency cycle at `{name}`"));
        }
        if self.visiting.len() >= 64 { return Err("motion dependencies exceed 64 levels".into()); }
        self.visiting.insert(name.to_string());
        let (st,d) = self.decls[name];
        let result = (|| {
            let mut measured = Vec::new();
            let def = match d.motion.as_ref().ok_or("a motion needs its defining relationship")? {
                MotionSpec::Rotation {axis,ratio,phase,advance} => {
                    let e = self.res.lookup(axis)
                        .ok_or_else(|| format!("no such motion axis: `{}`",axis.root.text))?;
                    let e = follow_building(self.sk,self.res,e,axis)?;
                    if e.kind != EntKind::Line || e.i() >= self.sk.lines.len() {
                        return Err("a motion rotates about a directed line".into());
                    }
                    measured.extend(self.measured(ratio,MotionSlot::Ratio)?);
                    measured.extend(self.measured(phase,MotionSlot::Phase)?);
                    measured.extend(self.measured(advance,MotionSlot::Advance)?);
                    MotionDef::Rotation {axis:e.idx,
                        ratio:self.number(ratio,1.,Dim::SCALAR,"ratio")?,
                        phase:self.number(phase,0.,Dim::ANGLE,"phase")?.to_radians(),
                        advance:self.number(advance,0.,Dim::LENGTH,"advance")?}
                }
                MotionSpec::Translation {axis,advance} => {
                    let e = self.res.lookup(axis)
                        .ok_or_else(|| format!("no such motion axis: `{}`",axis.root.text))?;
                    let e = follow_building(self.sk,self.res,e,axis)?;
                    if e.kind != EntKind::Line || e.i() >= self.sk.lines.len() {
                        return Err("a motion translates along a directed line".into());
                    }
                    let advance = Some(advance.clone());
                    measured.extend(self.measured(&advance,MotionSlot::Advance)?);
                    MotionDef::Translation {axis:e.idx,
                        advance:self.number(&advance,0.,Dim::LENGTH,"advance")?}
                }
                MotionSpec::Relative {source,observer} => MotionDef::Relative {
                    source:self.dependency(source)? as u32,observer:self.dependency(observer)? as u32,
                },
            };
            let height = match def {
                MotionDef::Rotation {..} | MotionDef::Translation {..} => 1,
                MotionDef::Relative {source,observer} =>
                    1+self.heights[source as usize].max(self.heights[observer as usize]),
            };
            if height > 64 { return Err("motion dependencies exceed 64 levels".into()); }
            self.heights.push(height);
            let i = self.sk.motions.len();
            self.sk.motions.push(MotionE {def,name:name.to_string(),class:d.class.clone(),measured});
            let e = EntRef::new(EntKind::Motion,i);
            self.res.of.insert(name.to_string(),e);
            self.map.bind(name,e,d.name.named());
            self.map.record(st,Made::Ent(e));
            Ok(i)
        })();
        self.visiting.remove(name);
        self.done.insert(name.to_string(),result.clone());
        result
    }
}

pub(super) fn motions(sk: &mut Sketch, res: &mut Resolver, map: &mut SourceMap,
    body: &[&Statement], skip: &BTreeSet<StmtId>, diags: &mut Vec<Diag>) {
    let decls: BTreeMap<_,_> = body.iter().filter_map(|&st| {
        let Operation::Decl(d) = &st.kind else { return None; };
        (d.kind == EntKind::Motion && !skip.contains(&st.id))
            .then(|| (d.name.key().text.clone(),(st,d.as_ref())))
    }).collect();
    let names: Vec<_> = decls.keys().cloned().collect();
    let mut b = Builder {sk,res,map,decls,done:BTreeMap::new(),visiting:BTreeSet::new(),heights:vec![]};
    for name in names {
        if let Err(message) = b.build(&name) {
            let st = b.decls[&name].0;
            diags.push(Diag {code:Code::E080,span:st.span,stmt:Some(st.id),message});
            b.res.of.remove(&name);
        }
    }
}
