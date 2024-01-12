use std::{path::PathBuf, fs, error::Error};

use cargo_metadata::MetadataCommand;

// FROM https://github.com/rust-lang/cargo/issues/9661
pub fn get_cargo_target_dir() -> Result<std::path::PathBuf, Box<dyn std::error::Error>> {
  let out_dir = std::path::PathBuf::from(std::env::var("OUT_DIR")?);
  let profile = std::env::var("PROFILE")?;
  let mut target_dir = None;
  let mut sub_path = out_dir.as_path();
  while let Some(parent) = sub_path.parent() {
    if parent.ends_with(&profile) {
      target_dir = Some(parent);
      break;
    }
    sub_path = parent;
  }
  let target_dir = target_dir.ok_or("not found")?;
  Ok(target_dir.to_path_buf())
}

pub enum LibraryType {
  Link,
  Runtime
}

pub fn identify_libraries(target: &str, lt: LibraryType) -> Result<Vec<PathBuf>, Box<dyn Error>> {
  let path = std::env::var("CARGO_MANIFEST_DIR")?;
  let meta = MetadataCommand::new().manifest_path("./Cargo.toml").current_dir(&path).exec()?;

  let mut library_paths = vec![];

  for package in meta.packages {
    if let Some(meta) = package.metadata.as_object() {
      if let Some(robot_rs_meta) = meta.get("robot_rs") {
        if let Some(vendor_dir) = robot_rs_meta.get("vendor_dir") {
          for dir in vendor_dir.as_array().unwrap() {
            let vendor_path = package.manifest_path.parent().unwrap().join(dir.as_str().unwrap());
            let libs_dir = vendor_path.join(target).join(match lt {
              LibraryType::Link => "libs",
              LibraryType::Runtime => "runtime-libs",
            });

            match fs::read_dir(libs_dir) {
              Ok(paths) => for p in paths {
                match p {
                  Ok(p) => {
                    library_paths.push(p.path());
                  },
                  Err(_) => (),
                }
              },
              Err(_) => ()    // No libs for this target
            }
          }
        }
      }
    }
  }

  Ok(library_paths)
}

pub fn copy_deps(libraries: Vec<PathBuf>) -> Result<(), Box<dyn Error>> {
  let depdir = get_cargo_target_dir()?.join("deps");

  eprintln!("{:?}", depdir);

  for p in libraries {
    fs::copy(p.clone(), &depdir.join(p.file_name().unwrap()))?;
  }

  Ok(())
}