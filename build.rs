use std::env;
use std::path::{Path, PathBuf};

fn main() {
    // rza1-rt puts `link.x` on the linker search path and deluge-bsp puts the Deluge's
    // `memory.x` next to it; this selects the script. Host (simulator) builds use the
    // platform's own linker setup.
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("none") {
        println!("cargo:rustc-link-arg-bins=-Tlink.x");
    }

    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let embedded = env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("none");
    let arm = env::var("CARGO_CFG_TARGET_FEATURE")
        .map(|f| f.split(',').any(|feat| feat == "neon"))
        .unwrap_or(false);
    build_smol_core(&manifest_dir, embedded, arm);
}

/// Compile `csrc/additive.cpp` (the C++/Argon additive engine) into the app.
fn build_smol_core(manifest_dir: &Path, embedded: bool, arm: bool) {
    // Locate the header-only Argon library.

    let source1 = manifest_dir.join("csrc/smol_reader.c");
    let source2 = manifest_dir.join("csrc/smol_process.c");

    let mut build = cc::Build::new();
    build
        .std("c11")
        .file(&source1)
        .file(&source2)
        .cpp_link_stdlib(None)
        .opt_level(2)
        .warnings(false);

    if embedded {
        build
            .compiler("arm-none-eabi-gcc")
            .archiver("arm-none-eabi-ar")
            .flag("-mcpu=cortex-a9")
            .flag("-mfloat-abi=hard")
            .flag("-mfpu=neon-vfpv3")
            .flag("-ffunction-sections")
            .flag("-fdata-sections")
            .flag("-fno-common")
            .flag("-ffreestanding")
            ;
    } else if arm {
        build.flag("-mfpu=neon-vfpv3");
    } else {
        // Host (simulator): no `__ARM_NEON`, so Argon includes SIMDe's
    }

    build.compile("smol");

    println!("cargo:rerun-if-changed={}", source1.display());
    println!("cargo:rerun-if-changed={}", source2.display());
}
