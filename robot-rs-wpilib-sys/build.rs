extern crate bindgen;

const SYMBOL_REGEX: &str = r"(HAL_)\w+";

use std::{env, path::PathBuf};

fn main() {
  let target = env::var("TARGET").unwrap();

  let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap()).canonicalize().unwrap();
  // let link = Path::join(Path::new(&out_dir), "libnv.so");
  let vendor_libs_dir = PathBuf::from(format!("vendor/{}/libs", target)).canonicalize().unwrap();
  let vendor_out_dir = out_dir.join("libs");
  std::fs::create_dir_all(&vendor_out_dir).ok();

  // Copy the libraries - the NI libraries have version numbers after the .so that the link can't pick up.
  println!("cargo:rustc-link-search={}", vendor_out_dir.to_str().unwrap());
  for entry in std::fs::read_dir(vendor_libs_dir).unwrap() {
    let de = entry.unwrap();
    let mut name = de.file_name().to_os_string().into_string().unwrap();
    
    match target.as_str() {
      "arm-unknown-linux-gnueabi" => {
        let mut split = name.split(".so");
        name = split.next().unwrap().to_owned() + ".so";
      },
      _ => ()
    }

    std::fs::copy(de.path(), vendor_out_dir.join(name)).unwrap();
  }

  // Import bindings from WPI libraries
  // println!("cargo:rustc-link-search={}", vendor_libs_dir.to_str().unwrap());
  let profile = std::env::var("PROFILE").unwrap();
  match profile.as_str() {
      "debug" => {
        println!("cargo:rustc-link-lib=wpiHald");
        println!("cargo:rustc-link-lib=wpiutild");
      },
      _ => {
        println!("cargo:rustc-link-lib=wpiHal");
        println!("cargo:rustc-link-lib=wpiutil");
      },
  }
  match target.as_str() {
    "arm-unknown-linux-gnueabi" => {
      println!("cargo:rustc-link-lib=FRC_NetworkCommunication");
      println!("cargo:rustc-link-lib=RoboRIO_FRC_ChipObject");
      println!("cargo:rustc-link-lib=visa");
      println!("cargo:rustc-link-lib=embcanshim");
      println!("cargo:rustc-link-lib=fpgalvshim");
    },
    _ => ()
  };
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