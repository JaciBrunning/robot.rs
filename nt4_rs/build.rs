use std::{fs, path::PathBuf, process::Command, str};

use anyhow::Context;
use cargo_metadata::MetadataCommand;

fn get_target() -> anyhow::Result<String> {
  let output = Command::new("rustc")
    .arg("-vV")
    .output()
    .context("Failed to run rustc to get the host target")?;
  let output = str::from_utf8(&output.stdout).context("`rustc -vV` didn't return utf8 output")?;

  let field = "host: ";
  let host = output
    .lines()
    .find(|l| l.starts_with(field))
    .map(|l| &l[field.len()..])
    .ok_or_else(|| {
        anyhow::format_err!(
            "`rustc -vV` didn't have a line for `{}`, got:\n{}",
            field.trim(),
            output
        )
    })?
    .to_string();
  Ok(host)
}

fn copy_deps() {
  let path = std::env::var("CARGO_MANIFEST_DIR").unwrap();
  let meta = MetadataCommand::new().manifest_path("./Cargo.toml").current_dir(&path).exec().unwrap();

  for package in meta.packages {
    if let Some(meta) = package.metadata.as_object() {
      if let Some(robot_rs_meta) = meta.get("robot_rs") {
        if let Some(libs_dir) = robot_rs_meta.get("libs_dir") {
          let dirs = libs_dir.as_array().unwrap();
          for dir in dirs {
            let path = package.manifest_path.parent().unwrap().join(dir.as_str().unwrap());
            let outdir = PathBuf::from(std::env::var("OUT_DIR").unwrap());
            let deps_dir = outdir.parent().unwrap().parent().unwrap().parent().unwrap().join("deps");

            for p in fs::read_dir(path).unwrap() {
              println!("{:?} {:?}", p, deps_dir);
              let p = p.unwrap();
              fs::copy(p.path(), &deps_dir.join(p.file_name())).unwrap();
            }
          }
        }
      }
    }
  }
}

fn main() {
  copy_deps();
  
  let target = get_target().unwrap_or("unknown".to_string());
  match target.as_str() {
    "arm-unknown-linux-gnueabi" | "armv7-unknown-linux-gnueabi" => println!("cargo:rustc-cfg=roborio"),
    _ => println!("cargo:rustc-cfg=simulation")
  }
}