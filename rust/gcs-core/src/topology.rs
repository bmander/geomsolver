//! Coordinate-free topology for one closed, oriented shell. Vertices and edges
//! are identities supplied by the caller, never coordinates welded by tolerance.
//! This establishes connectivity and manifoldness, not geometric embedding,
//! surface incidence, self-intersection freedom, outward normals or export error.
use std::collections::BTreeMap;

#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Direction { Forward, Reverse }

impl Direction {
    pub fn reversed(self) -> Self {
        match self { Self::Forward => Self::Reverse, Self::Reverse => Self::Forward }
    }
    fn endpoints(self) -> [usize;2] {
        match self { Self::Forward => [0,1], Self::Reverse => [1,0] }
    }
}

#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub struct EdgeUse {
    pub edge: usize,
    pub direction: Direction,
}

/// An oriented connected face, topologically a disk with optional holes. Every
/// boundary is an ordered loop of directed uses. The geometric realization of
/// that face and those loops must be checked separately. Periodic surfaces may
/// use one edge twice, in opposite directions, on the same face.
#[derive(Clone,Debug)]
pub struct Face {
    pub loops: Vec<Vec<EdgeUse>>,
}

#[derive(Clone,Debug,PartialEq,Eq)]
pub enum Error {
    EmptyShell,
    VertexOutOfRange {edge: usize,vertex: usize},
    EdgeOutOfRange {face: usize,edge: usize},
    EmptyFace {face: usize},
    EmptyLoop {face: usize,boundary: usize},
    OpenLoop {face: usize,boundary: usize,after: usize},
    EdgeUseCount {edge: usize,count: usize},
    InconsistentOrientation {edge: usize},
    UnusedVertices,
    NonManifoldVertex {vertex: usize},
    DisconnectedShell,
    InvalidEulerCharacteristic,
    DegenerateTriangle {face: usize},
}

/// An immutable, checked closed orientable 2-manifold. Geometry is deliberately
/// absent: passing this check alone does not make a set of patches a valid solid.
#[derive(Clone,Debug)]
pub struct ClosedShell {
    vertices: usize,
    edges: Vec<[usize;2]>,
    faces: Vec<Face>,
    edge_faces: Vec<[usize;2]>,
    euler: i64,
}

const NONE: usize = usize::MAX;

impl ClosedShell {
    pub fn new(vertices: usize,edges: Vec<[usize;2]>,faces: Vec<Face>) -> Result<Self,Error> {
        if vertices == 0 || edges.is_empty() || faces.is_empty() { return Err(Error::EmptyShell); }
        for (edge,ends) in edges.iter().enumerate() {
            for &vertex in ends {
                if vertex >= vertices { return Err(Error::VertexOutOfRange {edge,vertex}); }
            }
        }
        // Bound allocations by the supplied edge data, even for an untrusted vertex count.
        let darts = edges.len().checked_mul(2).ok_or(Error::UnusedVertices)?;
        if vertices > darts { return Err(Error::UnusedVertices); }
        let mut incident = vec![0usize;vertices];
        let mut first = vec![NONE;vertices];
        for (e,ends) in edges.iter().enumerate() {
            for k in 0..2 { incident[ends[k]] += 1; first[ends[k]] = 2*e+k; }
        }
        if incident.contains(&0) { return Err(Error::UnusedVertices); }
        let mut edge_faces = vec![[NONE;2];edges.len()];
        let mut balance = vec![0i8;edges.len()];
        // The link of a vertex has one node per incident edge *end*. Keeping ends
        // distinct also handles a closed edge whose two endpoints are one vertex.
        let mut links = vec![[NONE;2];darts];
        let mut degree = vec![0usize;darts];
        let mut face_euler = 0i64;
        for (face,f) in faces.iter().enumerate() {
            if f.loops.is_empty() { return Err(Error::EmptyFace {face}); }
            face_euler += 2-f.loops.len() as i64;
            for (boundary,uses) in f.loops.iter().enumerate() {
                if uses.is_empty() { return Err(Error::EmptyLoop {face,boundary}); }
                for u in uses {
                    if u.edge >= edges.len() { return Err(Error::EdgeOutOfRange {face,edge:u.edge}); }
                    let owners = &mut edge_faces[u.edge];
                    let slot = if owners[0] == NONE { 0 } else { 1 };
                    if owners[slot] != NONE { return Err(Error::EdgeUseCount {edge:u.edge,count:3}); }
                    owners[slot] = face;
                    balance[u.edge] += if u.direction == Direction::Forward { 1 } else { -1 };
                }
                for i in 0..uses.len() {
                    let a = uses[i]; let b = uses[(i+1)%uses.len()];
                    let a = 2*a.edge+a.direction.endpoints()[1];
                    let b = 2*b.edge+b.direction.endpoints()[0];
                    let vertex = edges[a/2][a%2];
                    if vertex != edges[b/2][b%2] {
                        return Err(Error::OpenLoop {face,boundary,after:i});
                    }
                    // One face corner contributes an undirected link edge. Parallel
                    // edges and self-loops are intentional in valid small cellulations.
                    for (p,q) in [(a,b),(b,a)] {
                        if degree[p] == 2 { return Err(Error::NonManifoldVertex {vertex}); }
                        links[p][degree[p]] = q; degree[p] += 1;
                    }
                }
            }
        }
        for (edge,owners) in edge_faces.iter().enumerate() {
            let count = owners.iter().filter(|&&f| f != NONE).count();
            if count != 2 { return Err(Error::EdgeUseCount {edge,count}); }
            if balance[edge] != 0 { return Err(Error::InconsistentOrientation {edge}); }
        }
        // Paired edges are insufficient: several closed surface fans can meet at
        // one vertex. Every vertex link must be one circle, not several circles.
        let mut visited = vec![false;darts];
        let mut stack = vec![];
        for vertex in 0..vertices {
            stack.push(first[vertex]);
            let mut reached = 0;
            while let Some(p) = stack.pop() {
                if visited[p] { continue; }
                visited[p] = true; reached += 1;
                if degree[p] != 2 { return Err(Error::NonManifoldVertex {vertex}); }
                stack.extend(links[p].iter().copied().filter(|&q| !visited[q]));
            }
            if reached != incident[vertex] { return Err(Error::NonManifoldVertex {vertex}); }
        }
        let mut visited = vec![false;faces.len()];
        stack.push(0);
        let mut reached = 0;
        while let Some(f) = stack.pop() {
            if visited[f] { continue; }
            visited[f] = true; reached += 1;
            for u in faces[f].loops.iter().flatten() {
                stack.extend(edge_faces[u.edge].iter().copied().filter(|&q| !visited[q]));
            }
        }
        if reached != faces.len() { return Err(Error::DisconnectedShell); }
        // Each connected genus-zero face with b boundary loops contributes 2-b,
        // so an annular face contributes zero rather than being counted as a disk.
        let euler = vertices as i64-edges.len() as i64+face_euler;
        if euler > 2 || euler % 2 != 0 { return Err(Error::InvalidEulerCharacteristic); }
        Ok(Self {vertices,edges,faces,edge_faces,euler})
    }

    /// Index identities determine shared edges. This adapter performs no coordinate
    /// welding, T-junction repair or geometry checks on the supplied triangles.
    pub fn from_triangles(vertices: usize,triangles: &[[usize;3]]) -> Result<Self,Error> {
        let mut edge_ids = BTreeMap::new();
        let mut edges = vec![];
        let mut faces = Vec::with_capacity(triangles.len());
        for (face,&[a,b,c]) in triangles.iter().enumerate() {
            if a == b || b == c || c == a { return Err(Error::DegenerateTriangle {face}); }
            let mut uses = Vec::with_capacity(3);
            for (a,b) in [(a,b),(b,c),(c,a)] {
                let ends = [a.min(b),a.max(b)];
                let edge = *edge_ids.entry(ends).or_insert_with(|| {
                    let e = edges.len(); edges.push(ends); e
                });
                uses.push(EdgeUse {edge,direction:if a < b { Direction::Forward } else { Direction::Reverse }});
            }
            faces.push(Face {loops:vec![uses]});
        }
        Self::new(vertices,edges,faces)
    }

    pub fn vertex_count(&self) -> usize { self.vertices }
    pub fn edges(&self) -> &[[usize;2]] { &self.edges }
    pub fn faces(&self) -> &[Face] { &self.faces }
    pub fn incident_faces(&self,edge: usize) -> Option<[usize;2]> { self.edge_faces.get(edge).copied() }
    pub fn loop_vertices(&self,face: usize,boundary: usize)
        -> Option<impl ExactSizeIterator<Item=usize> + '_> {
        let uses = self.faces.get(face)?.loops.get(boundary)?;
        Some(uses.iter().map(move |u| self.edges[u.edge][u.direction.endpoints()[0]]))
    }
    pub fn euler_characteristic(&self) -> i64 { self.euler }
    pub fn genus(&self) -> usize { ((2-self.euler)/2) as usize }
}
