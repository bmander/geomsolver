use std::{env,path::PathBuf,process::Command};

fn run(command: &mut Command) {
    assert!(command.status().expect("native CAD build command starts").success(),
        "native CAD build failed: {command:?}");
}

fn main() {
    println!("cargo:rerun-if-changed=backend/occt.cpp");
    for name in ["OCCT_ROOT","CXX","AR"] { println!("cargo:rerun-if-env-changed={name}"); }
    if env::var_os("CARGO_FEATURE_OCCT").is_none() { return; }
    let target = env::var("TARGET").unwrap();
    assert_eq!(target,env::var("HOST").unwrap(),"OCCT currently requires a native host build");
    let root = env::var_os("OCCT_ROOT").map(PathBuf::from).unwrap_or_else(|| {
        ["/usr/local/opt/opencascade","/opt/homebrew/opt/opencascade","/usr"]
            .into_iter().map(PathBuf::from).find(|p| p.join("include/opencascade").is_dir())
            .expect("install Open CASCADE development files or set OCCT_ROOT")
    });
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let object = out.join("occt.o");
    run(Command::new(env::var_os("CXX").unwrap_or_else(|| "c++".into()))
        .args(["-std=c++17","-O2","-fPIC","-c","backend/occt.cpp","-o"])
        .arg(&object).arg("-I").arg(root.join("include/opencascade")));
    run(Command::new(env::var_os("AR").unwrap_or_else(|| "ar".into()))
        .arg("crs").arg(out.join("libsolvent_occt.a")).arg(object));
    println!("cargo:rustc-link-search=native={}",out.display());
    println!("cargo:rustc-link-lib=static=solvent_occt");
    println!("cargo:rustc-link-search=native={}",root.join("lib").display());
    for library in ["TKDESTEP","TKDESTL","TKMesh","TKXSBase","TKBO","TKBool","TKPrim","TKTopAlgo",
        "TKGeomAlgo","TKBRep","TKGeomBase","TKG3d","TKG2d","TKMath","TKernel"] {
        println!("cargo:rustc-link-lib=dylib={library}");
    }
    println!("cargo:rustc-link-lib={}",if target.contains("apple") { "c++" } else { "stdc++" });
    println!("cargo:rustc-link-arg=-Wl,-rpath,{}",root.join("lib").display());
}
