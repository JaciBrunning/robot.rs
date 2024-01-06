extern crate bindgen;

const SYMBOL_REGEX: &str = r"(HAL_)\w+";

use std::{env, path::PathBuf};

fn main() {
  let target = env::var("TARGET").unwrap();

  // Import bindings from WPI libraries
  println!("cargo:rustc-link-search={}", PathBuf::from(format!("vendor/{}/libs", target)).canonicalize().unwrap().to_str().unwrap());
  let profile = std::env::var("PROFILE").unwrap();
  match profile.as_str() {
      "debug" => {
        println!("cargo:rustc-link-lib=wpiHald");
        println!("cargo:rustc-link-lib=wpiutild");
        println!("cargo:rustc-link-lib=ntcored");
      },
      _ => {
        println!("cargo:rustc-link-lib=wpiHal");
        println!("cargo:rustc-link-lib=wpiutil");
        println!("cargo:rustc-link-lib=ntcore");
      },
  }
  println!("cargo:rerun-if-changed=HALWrapper.h");

  // Some config copied from first-rust-competition https://github.com/first-rust-competition/first-rust-competition/blob/master/hal-gen/src/main.rs
  let bindings = bindgen::Builder::default()
    .header("HALWrapper.h")
    .derive_default(true)
    .clang_arg(format!("-Ivendor/{}/headers", target))
    .allowlist_type(SYMBOL_REGEX)
    .allowlist_function(SYMBOL_REGEX)
    .allowlist_var(SYMBOL_REGEX)
    .default_enum_style(bindgen::EnumVariation::Rust { non_exhaustive: false })
    .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
    .clang_args(&[
      format!("--target={}", target)    // See: https://github.com/rust-lang/rust-bindgen/issues/1760
    ])
    .generate()
    .expect("Unable to generate bindings");

  let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
  bindings
    .write_to_file(out_path.join("bindings.rs"))
    .expect("Couldn't write bindings!");
}