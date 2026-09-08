//! Opt-in external-mesher comparison, outside the shipped Solvent workspace.
mod fixtures;
use fidget::{mesh::{Octree,Settings},vm::VmShape};
use std::{path::PathBuf,time::Instant};

fn main() -> Result<(),Box<dyn std::error::Error>> {
    let args:Vec<_> = std::env::args().skip(1).collect();
    if args.len() != 3 {
        return Err("usage: solvent-implicit-mesh-bench CASE|all DEPTH OUTPUT_DIRECTORY".into());
    }
    let depth:u8 = args[1].parse()?;
    if !(2..=10).contains(&depth) { return Err("depth must be between 2 and 10".into()); }
    let names:Vec<&str> = if args[0] == "all" { fixtures::NAMES.to_vec() } else { vec![&args[0]] };
    if names.iter().any(|name| !fixtures::NAMES.contains(name)) { return Err("unknown fixture".into()); }
    let output = PathBuf::from(&args[2]); std::fs::create_dir_all(&output)?;
    for name in names {
        let start = Instant::now();
        let (tree,extent) = fixtures::build(name);
        let shape = VmShape::from(tree);
        let bound = shape.try_into()?;
        let setup_seconds = start.elapsed().as_secs_f64();
        // Serial measurements avoid worker-startup and thread-count differences.
        let mut settings = Settings {depth,threads:None,..Default::default()};
        for k in 0..3 { settings.world_to_model[(k,k)] = extent; }
        let start = Instant::now();
        let octree = Octree::build(&bound,&settings).ok_or("meshing cancelled")?;
        let mesh = octree.walk_dual();
        let extraction_seconds = start.elapsed().as_secs_f64();
        let stem = format!("{name}-depth{depth}");
        let mut file = std::fs::File::create(output.join(format!("{stem}.stl")))?;
        mesh.write_stl(&mut file)?;
        let positions:Vec<_> = mesh.vertices.iter().map(|v| [v.x,v.y,v.z]).collect();
        let triangles:Vec<_> = mesh.triangles.iter().map(|t| [t.x,t.y,t.z]).collect();
        let report = serde_json::json!({
            "case":name,"backend":"fidget-0.5.0-vm","depth":depth,"extent":extent,
            "setup_seconds":setup_seconds,"extraction_seconds":extraction_seconds,
            "threads":1,"point_queries":null,"status":"unchecked_expression_baseline",
            "vertices":positions,"triangles":triangles,
            "scope":"Closed-form baseline, not the Solvent MaterialField adapter or an accepted solid."
        });
        std::fs::write(output.join(format!("{stem}.json")),serde_json::to_vec(&report)?)?;
        println!("{name} depth {depth}: {} triangles, {:.3} ms extraction, {:.3} ms setup",
            triangles.len(),extraction_seconds*1000.,setup_seconds*1000.);
    }
    Ok(())
}
