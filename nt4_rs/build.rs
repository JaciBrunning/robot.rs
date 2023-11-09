use std::{fs, path::PathBuf, process::Command, str, path::Path};

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

fn copy_directory_contents(src: &Path, dest: &Path) {
  // Ensure the destination directory exists.
  let _ = fs::create_dir_all(dest);

  for entry in fs::read_dir(src).unwrap() {
      let entry = entry.unwrap();

      let entry_path = entry.path();
      let dest_path = dest.join(entry_path.file_name().unwrap());

      if entry_path.is_file() {
          // If the entry is a file, copy it to the destination.
          let _ = fs::copy(&entry_path, &dest_path);
      } else if entry_path.is_dir() {
          // If the entry is a directory, recursively copy its contents.
          copy_directory_contents(&entry_path, &dest_path);
      }
  }
}

fn copy_deps() {
    let path = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let meta = MetadataCommand::new()
        .manifest_path("./Cargo.toml")
        .current_dir(&path)
        .exec()
        .unwrap();

    for package in meta.packages {
        if let Some(meta) = package.metadata.as_object() {
            if let Some(robot_rs_meta) = meta.get("robot_rs") {
                if let Some(libs_dir) = robot_rs_meta.get("libs_dir") {
                    let dirs = libs_dir.as_array().unwrap();
                    for dir in dirs {
                        let path = package
                            .manifest_path
                            .parent()
                            .unwrap()
                            .join(dir.as_str().unwrap());
                        let outdir = PathBuf::from(std::env::var("OUT_DIR").unwrap());
                        let deps_dir = outdir
                            .parent()
                            .unwrap()
                            .parent()
                            .unwrap()
                            .parent()
                            .unwrap()
                            .join("deps");

                            for entry in fs::read_dir(path).unwrap() {
                              let entry = entry.unwrap();
                              let entry_path = entry.path();
                              if entry_path.is_file() {
                                  println!("{:?} {:?}", entry_path, deps_dir);
                                  fs::copy(&entry_path, &deps_dir.join(entry_path.file_name().unwrap())).unwrap();
                              } else if entry_path.is_dir() {
                                  let dest_dir = deps_dir.join(entry_path.file_name().unwrap());
                                  fs::create_dir_all(&dest_dir).unwrap();
                                  copy_directory_contents(&entry_path, &dest_dir);
                              }
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
        "arm-unknown-linux-gnueabi" | "armv7-unknown-linux-gnueabi" => {
            println!("cargo:rustc-cfg=roborio")
        }
        _ => println!("cargo:rustc-cfg=simulation"),
    }
}