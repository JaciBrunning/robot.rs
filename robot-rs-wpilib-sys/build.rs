extern crate bindgen;

const SYMBOL_REGEX: &str = r"(HAL_)\w+";

use std::{env, path::PathBuf};

use robot_rs_build_utils::{get_meta, link_against, maybe_download_libs, Profile};

fn main() {
  let target = env::var("TARGET").unwrap();
  let profile_str = std::env::var("PROFILE").unwrap();
  let profile = match profile_str.as_str() {
    "debug" => Profile::Debug,
    _ => Profile::Release
  };

  let (header_dirs, linktime_dirs, _runtime_dirs) = maybe_download_libs(get_meta().unwrap().root_package().unwrap(), target.as_str(), profile).unwrap();
  link_against(target.as_str(), linktime_dirs).unwrap();

  // Needed to always download libs
  println!("cargo:rerun-if-changed=NULL");

  // Some config copied from first-rust-competition https://github.com/first-rust-competition/first-rust-competition/blob/master/hal-gen/src/main.rs
  let bindings = bindgen::Builder::default()
    .header("HALWrapper.h")
    .derive_default(true)
    .clang_args(header_dirs.iter().map(|x| format!("-I{}", x.as_os_str().to_string_lossy())))
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