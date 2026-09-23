use std::{env,path::PathBuf,process::Command};

fn run(command: &mut Command) {
    assert!(command.status().expect("native CAD build command starts").success(),
        "native CAD build failed: {command:?}");
}

fn main() {
    for file in ["occt.cpp","boundary.cpp","surfaces.cpp","trims.cpp","cells.cpp","sections.cpp","occt.hpp"] {
        println!("cargo:rerun-if-changed=backend/{file}");
    }
    for name in ["OCCT_ROOT","MANIFOLD_ROOT","CXX","AR"] { println!("cargo:rerun-if-env-changed={name}"); }
    if env::var_os("CARGO_FEATURE_MANIFOLD").is_some() {
        // Manifold's C binding is a plain dynamic library; nothing is compiled here.
        let root = env::var_os("MANIFOLD_ROOT").map(PathBuf::from).unwrap_or_else(|| {
            ["/usr/local/opt/manifold","/opt/homebrew/opt/manifold","/usr"]
                .into_iter().map(PathBuf::from).find(|p| p.join("include/manifold/manifoldc.h").is_file())
                .expect("install the Manifold library (brew install manifold) or set MANIFOLD_ROOT")
        });
        println!("cargo:rustc-link-search=native={}",root.join("lib").display());
        println!("cargo:rustc-link-lib=dylib=manifoldc");
        println!("cargo:rustc-link-arg=-Wl,-rpath,{}",root.join("lib").display());
    }
    if env::var_os("CARGO_FEATURE_CGAL").is_some() {
        // Header-only CGAL (Homebrew's, or CGAL_ROOT), Boost headers, and GMP/MPFR.
        println!("cargo:rerun-if-changed=backend/delpsc.cpp");
        println!("cargo:rerun-if-env-changed=CGAL_ROOT");
        let find = |name: &str,probe: &str| -> PathBuf {
            ["/usr/local/opt","/opt/homebrew/opt"].into_iter().map(|p| PathBuf::from(p).join(name))
                .find(|p| p.join(probe).exists()).unwrap_or_else(|| panic!("install {name} (brew install cgal)"))
        };
        let cgal = env::var_os("CGAL_ROOT").map(PathBuf::from).unwrap_or_else(|| find("cgal","include/CGAL/make_mesh_3.h"));
        let boost = find("boost","include/boost/version.hpp");
        let gmp = find("gmp","include/gmp.h");
        let mpfr = find("mpfr","include/mpfr.h");
        let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
        let object = out.join("delpsc.o");
        let mut compile = Command::new(env::var_os("CXX").unwrap_or_else(|| "c++".into()));
        compile.args(["-std=c++17","-O2","-fPIC","-frounding-math","-DCGAL_NDEBUG","-c","backend/delpsc.cpp","-o"]).arg(&object);
        for root in [&cgal,&boost,&gmp,&mpfr] { compile.arg("-I").arg(root.join("include")); }
        run(&mut compile);
        run(Command::new(env::var_os("AR").unwrap_or_else(|| "ar".into())).arg("crs").arg(out.join("libsolvent_cgal.a")).arg(&object));
        println!("cargo:rustc-link-search=native={}",out.display());
        println!("cargo:rustc-link-lib=static=solvent_cgal");
        for root in [&gmp,&mpfr] { println!("cargo:rustc-link-search=native={}",root.join("lib").display()); }
        println!("cargo:rustc-link-lib=dylib=gmp");
        println!("cargo:rustc-link-lib=dylib=mpfr");
        println!("cargo:rustc-link-lib={}",if env::var("TARGET").unwrap().contains("apple") { "c++" } else { "stdc++" });
    }
    if env::var_os("CARGO_FEATURE_OCCT").is_none() { return; }
    let target = env::var("TARGET").unwrap();
    assert_eq!(target,env::var("HOST").unwrap(),"OCCT currently requires a native host build");
    let root = env::var_os("OCCT_ROOT").map(PathBuf::from).unwrap_or_else(|| {
        ["/usr/local/opt/opencascade","/opt/homebrew/opt/opencascade","/usr"]
            .into_iter().map(PathBuf::from).find(|p| p.join("include/opencascade").is_dir())
            .expect("install Open CASCADE development files or set OCCT_ROOT")
    });
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let mut objects = Vec::new();
    for name in ["occt","boundary","surfaces","trims","cells","sections"] {
        let object = out.join(format!("{name}.o"));
        run(Command::new(env::var_os("CXX").unwrap_or_else(|| "c++".into()))
            .args(["-std=c++17","-O2","-fPIC","-c"]).arg(format!("backend/{name}.cpp"))
            .arg("-o").arg(&object).arg("-I").arg(root.join("include/opencascade")));
        objects.push(object);
    }
    run(Command::new(env::var_os("AR").unwrap_or_else(|| "ar".into()))
        .arg("crs").arg(out.join("libsolvent_occt.a")).args(objects));
    println!("cargo:rustc-link-search=native={}",out.display());
    println!("cargo:rustc-link-lib=static=solvent_occt");
    println!("cargo:rustc-link-search=native={}",root.join("lib").display());
    for library in ["TKDESTEP","TKDESTL","TKMesh","TKShHealing","TKXSBase","TKBO","TKBool","TKPrim","TKTopAlgo",
        "TKGeomAlgo","TKBRep","TKGeomBase","TKG3d","TKG2d","TKMath","TKernel"] {
        println!("cargo:rustc-link-lib=dylib={library}");
    }
    println!("cargo:rustc-link-lib={}",if target.contains("apple") { "c++" } else { "stdc++" });
    println!("cargo:rustc-link-arg=-Wl,-rpath,{}",root.join("lib").display());
}
