//! Spatial search accelerates witness discovery, never the distance certificate.
use super::{distance_to_box,Error,I,V};

pub(super) struct WitnessIndex {
    bounds: V,
    vertex: usize,
    left: Option<Box<Self>>,
    right: Option<Box<Self>>,
}

impl WitnessIndex {
    pub(super) fn new(ids: &mut [usize],vertices: &[[f64;3]]) -> Option<Box<Self>> {
        if ids.is_empty() { return None; }
        let bounds = std::array::from_fn(|k| {
            let lo = ids.iter().map(|&i| vertices[i][k]).fold(f64::INFINITY,f64::min);
            let hi = ids.iter().map(|&i| vertices[i][k]).fold(f64::NEG_INFINITY,f64::max);
            I::new(lo,hi).unwrap()
        });
        let axis = (0..3).max_by(|&a,&b| {
            let width = |k:usize| { let [lo,hi] = bounds[k].bounds(); hi-lo };
            width(a).total_cmp(&width(b))
        }).unwrap();
        let mid = ids.len()/2;
        ids.select_nth_unstable_by(mid,|a,b| vertices[*a][axis].total_cmp(&vertices[*b][axis]).then(a.cmp(b)));
        let (left,tail) = ids.split_at_mut(mid);
        let (vertex,right) = tail.split_first_mut().unwrap();
        Some(Box::new(Self {bounds,vertex:*vertex,left:Self::new(left,vertices),right:Self::new(right,vertices)}))
    }

    // A lower bound on squared distance from any subtree point to any cell
    // point. It cannot exceed the farthest-corner distance we are minimizing.
    fn lower_squared(&self,cell: V) -> Result<f64,Error> {
        let mut sum = I::ZERO;
        for (a,b) in self.bounds.into_iter().zip(cell) {
            let [al,ah] = a.bounds(); let [bl,bh] = b.bounds();
            let gap = if al > bh { I::point(al)?.sub(I::point(bh)?)? }
                else if bl > ah { I::point(bl)?.sub(I::point(ah)?)? } else { I::ZERO };
            sum = sum.add(gap.square()?)?;
        }
        Ok(sum.bounds()[0])
    }

    pub(super) fn search(&self,cell: V,vertices: &[[f64;3]],tolerance: f64,
        best: &mut Option<(usize,f64)>) -> Result<(),Error> {
        let limit = best.map_or(tolerance,|(_,d)| d.min(tolerance));
        if self.lower_squared(cell)? > I::point(limit)?.square()?.bounds()[1] { return Ok(()); }
        let distance = distance_to_box(vertices[self.vertex],cell)?;
        if best.is_none_or(|(_,d)| distance < d) { *best = Some((self.vertex,distance)); }
        let mut children = [self.left.as_deref(),self.right.as_deref()];
        if let [Some(a),Some(b)] = children {
            if a.lower_squared(cell)? > b.lower_squared(cell)? { children.swap(0,1); }
        }
        for child in children.into_iter().flatten() { child.search(cell,vertices,tolerance,best)?; }
        Ok(())
    }
}
