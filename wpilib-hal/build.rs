extern crate bindgen;

use std::env;
use std::path::PathBuf;

const SYMBOL_REGEX: &str = r"(HAL_|NT_)\w+";

fn main() {
  println!("cargo:rustc-link-search={}", PathBuf::from("libs").canonicalize().unwrap().to_str().unwrap());
  println!("cargo:rustc-link-lib=wpiHal");
  println!("cargo:rustc-link-lib=wpiutil");
  println!("cargo:rustc-link-lib=ntcore");
  println!("cargo:rerun-if-changed=HALWrapper.h");

  // Some config copied from first-rust-competition https://github.com/first-rust-competition/first-rust-competition/blob/master/hal-gen/src/main.rs
  let bindings = bindgen::Builder::default()
    .header("HALWrapper.h")
    .derive_default(true)
    .clang_arg("-Iinclude")
    .whitelist_type(SYMBOL_REGEX)
    .whitelist_function(SYMBOL_REGEX)
    .whitelist_var(SYMBOL_REGEX)
    .default_enum_style(bindgen::EnumVariation::Rust { non_exhaustive: false })
    .parse_callbacks(Box::new(bindgen::CargoCallbacks))
    .generate()
    .expect("Unable to generate bindings");

  let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
  bindings
    .write_to_file(out_path.join("bindings.rs"))
    .expect("Couldn't write bindings!");
}