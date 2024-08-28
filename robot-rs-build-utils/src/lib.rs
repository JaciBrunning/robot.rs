use std::{
  error::Error,
  io::{Read, Write},
  path::{Path, PathBuf},
};

use cargo_metadata::{Metadata, MetadataCommand, Package};
use sha2::{Digest, Sha256};

#[derive(Debug, Clone, Copy)]
pub enum Profile {
  Debug,
  Release,
}

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

pub fn target_triple_to_wpi(target: &str) -> Result<[&'static str; 2], Box<dyn Error>> {
  match target {
    "arm-unknown-linux-gnueabi" => Ok(["linux", "athena"]),
    "arm-unknown-linux-gnueabihf" => Ok(["linux", "arm32"]),
    "aarch64-unknown-linux-gnu" => Ok(["linux", "arm64"]),
    "x86_64-unknown-linux-gnu" => Ok(["linux", "x86-64"]),
    "x64_64-apple-darwin" => Ok(["osx", "universal"]),
    "x86_64-pc-windows-msvc" => Ok(["windows", "x86-64"]),
    "aarch64-pc-windows-msvc" => Ok(["windows", "arm64"]),
    _ => Err(anyhow::anyhow!("Unknown Target: {}", target).into()),
  }
}

pub fn get_meta() -> Result<Metadata, Box<dyn Error>> {
  let path = std::env::var("CARGO_MANIFEST_DIR")?;
  let meta = MetadataCommand::new()
    .manifest_path("./Cargo.toml")
    .current_dir(&path)
    .exec()?;
  Ok(meta)
}

pub fn maybe_download_libs(
  package: &Package,
  target: &str,
  profile: Profile,
) -> Result<(Vec<PathBuf>, Vec<PathBuf>, Vec<PathBuf>), Box<dyn Error>> {
  let robot_rs_dir = home::home_dir()
    .ok_or(anyhow::anyhow!("Can't get your home directory :("))?
    .join(".robotrs");

  let download_folder = robot_rs_dir.join("mavenlibs");

  let [target_platform, architecture] = target_triple_to_wpi(target)?;
  let classifier_platform = [target_platform, architecture].join("");
  let classifier_suffix = match profile {
    Profile::Debug => "debug",
    Profile::Release => "",
  };

  let mut header_dirs = vec![];
  let mut runtime = vec![];
  let mut linktime = vec![];

  if let Some(robot_rs_key) = package.metadata.as_object().and_then(|m| m.get("robot_rs")) {
    let artifacts = robot_rs_key.get("artifacts").and_then(|x| x.as_array());
    let mavens = robot_rs_key.get("mavens").and_then(|x| x.as_object());

    match (artifacts, mavens) {
      (Some(artifacts), Some(mavens)) => {
        for item in artifacts.into_iter() {
          let maven = mavens
            .get(
              item
                .get("maven")
                .and_then(|x| x.as_str())
                .ok_or(anyhow::anyhow!("Artifact object without 'maven' key"))?,
            )
            .and_then(|x| x.as_str())
            .ok_or(anyhow::anyhow!("Missing Maven Repository"))?;
          let artifact = item
            .get("artifact")
            .and_then(|x| x.as_str())
            .ok_or(anyhow::anyhow!("Artifact object without 'artifact' key"))?;
          let only = item.get("only").and_then(|x| x.as_array());
          let no_headers = item
            .get("no_headers")
            .and_then(|x| x.as_bool())
            .unwrap_or(false);
          let link_only = item
            .get("link_only")
            .and_then(|x| x.as_bool())
            .unwrap_or(false);

          if let Some(only) = only {
            let mut found = false;
            for el in only.into_iter() {
              if el
                .as_str()
                .ok_or(anyhow::anyhow!("'only' values should be strings"))?
                == classifier_platform
              {
                found = true;
              }
            }
            if !found {
              continue;
            }
          }

          let mut split_artifact = artifact.split(":");
          let group = split_artifact
            .next()
            .ok_or(anyhow::anyhow!("Missing Artifact Group"))?;
          let name = split_artifact
            .next()
            .ok_or(anyhow::anyhow!("Missing Artifact Name"))?;
          let version = split_artifact
            .next()
            .ok_or(anyhow::anyhow!("Missing Artifact Version"))?;

          let fragment = format!(
            "{}/{}/{}/{}-{}-{}{}.zip",
            group.replace(".", "/"),
            name,
            version,
            name,
            version,
            classifier_platform,
            classifier_suffix
          );
          let header_fragment = format!(
            "{}/{}/{}/{}-{}-headers.zip",
            group.replace(".", "/"),
            name,
            version,
            name,
            version
          );

          let artifact_url = format!("{}/{}", maven, fragment);
          let headers_url = format!("{}/{}", maven, header_fragment);

          let extracted_dir = try_download_and_extract(
            &artifact_url,
            &download_folder.join(fragment),
            &robot_rs_dir,
          )?;
          let inner_path = extracted_dir
            .join(target_platform)
            .join(architecture)
            .join("shared");

          linktime.push(inner_path.clone());
          if !link_only {
            runtime.push(inner_path.clone());
          }

          if !no_headers {
            let extracted_dir = try_download_and_extract(
              &headers_url,
              &download_folder.join(header_fragment),
              &robot_rs_dir,
            )?;
            header_dirs.push(extracted_dir)
          }
        }
      }
      _ => (),
    }
  }
  Ok((header_dirs, linktime, runtime))
}

fn try_download_and_extract(
  url: &str,
  target_path: &Path,
  robot_rs_dir: &Path,
) -> Result<PathBuf, Box<dyn Error>> {
  let hash_file = Path::new(&format!(
    "{}.sha256",
    target_path.as_os_str().to_string_lossy()
  ))
  .to_owned();
  if !hash_file.exists() || !Path::new(target_path).exists() {
    println!("cargo:warning=Downloading: {}", url);
    let mut resp = reqwest::blocking::get(url)?;
    std::fs::create_dir_all(
      PathBuf::from(target_path)
        .parent()
        .unwrap()
        .as_os_str()
        .to_str()
        .unwrap(),
    )?;
    let mut f = std::fs::File::create(target_path)?;
    std::io::copy(&mut resp, &mut f)?;
  }

  if !hash_file.exists() {
    let mut hasher = Sha256::new();
    let mut f = std::fs::File::open(target_path)?;
    let mut buf = [0u8; 1024];
    loop {
      let n = f.read(&mut buf[..])?;
      if n == 0 {
        break;
      }

      hasher.update(&buf[0..n]);
    }
    let mut f_hash_out = std::fs::File::create(&hash_file)?;
    f_hash_out.write_all(format!("{:X}", hasher.finalize()).as_bytes())?;
  }

  let mut hashbuf = Vec::with_capacity(128);
  std::fs::File::open(&hash_file)?.read_to_end(&mut hashbuf)?;
  let hash = String::from_utf8(hashbuf)?;

  let extract_path = robot_rs_dir.join("extracted").join(hash);

  if !extract_path.exists() {
    match extract(target_path, &extract_path) {
      Ok(_) => Ok(extract_path),
      Err(e) => {
        if extract_path.exists() {
          std::fs::remove_dir_all(&extract_path)?;
        }
        Err(e)
      }
    }
  } else {
    Ok(extract_path)
  }
}

fn extract(target_path: &Path, extract_path: &Path) -> Result<(), Box<dyn Error>> {
  let file = std::fs::File::open(target_path)?;
  let mut zip = zip::ZipArchive::new(file)?;

  for i in 0..zip.len() {
    let mut inner_file = zip.by_index(i)?;
    if inner_file.is_file() {
      let inner_path = inner_file.enclosed_name().ok_or(anyhow::anyhow!(
        "Dangerous ZIP File - Path Traversal detected: {}",
        target_path.as_os_str().to_string_lossy()
      ))?;
      let joined_path = extract_path.join(inner_path);
      if !joined_path.exists() {
        std::fs::create_dir_all(joined_path.parent().unwrap().as_os_str().to_str().unwrap())?;
        let mut new_f = std::fs::File::create(joined_path.clone())?;
        std::io::copy(&mut inner_file, &mut new_f)?;
      }
    }
  }

  Ok(())
}

pub fn link_against(target: &str, linktime_dirs: Vec<PathBuf>) -> Result<(), Box<dyn Error>> {
  let renamed_dir = std::path::PathBuf::from(std::env::var("OUT_DIR")?).join("_renamed");
  std::fs::create_dir_all(&renamed_dir)?;
  println!("cargo:rustc-link-search={}", renamed_dir.to_string_lossy());

  let [target_platform, _] = target_triple_to_wpi(target)?;
  for dir in linktime_dirs {
    for path in std::fs::read_dir(&dir)? {
      let path = path?;
      let filename = path.file_name();
      let filename = filename.to_string_lossy();

      match target_platform {
        "windows" => {
          if filename.ends_with(".lib") {
            let libname = &filename[0..filename.len() - 4];
            println!("cargo:rustc-link-lib={}", libname);
            println!("cargo:rustc-link-search={}", dir.to_string_lossy());
          }
        }
        "linux" => {
          if filename.starts_with("lib") {
            if !filename.ends_with(".so")
              && !filename.ends_with(".so.debug")
              && filename.contains(".so")
            {
              // Need to copy the lib and drop the version suffix
              let mut split = filename.split(".so");
              let lib_name_with_lib = split.next().unwrap().to_owned();
              let new_filename = lib_name_with_lib.clone() + ".so";

              let path_to_new_file = renamed_dir.join(&new_filename);
              std::fs::copy(path.path(), path_to_new_file)?;
              println!("cargo:rustc-link-lib={}", &lib_name_with_lib[3..]);
            } else if filename.ends_with(".so") {
              let libname = &filename[3..filename.len() - 3];
              println!("cargo:rustc-link-lib={}", libname);
              println!("cargo:rustc-link-search={}", dir.to_string_lossy());
            }
          }
        }
        "osx" => {
          if filename.starts_with("lib") && filename.ends_with(".dylib") {
            let libname = &filename[3..filename.len() - 6];
            println!("cargo:rustc-link-lib={}", libname);
            println!("cargo:rustc-link-search={}", dir.to_string_lossy());
          }
        }
        _ => Err(anyhow::anyhow!("Unknown Platform: {}", target_platform))?,
      }
    }
  }
  Ok(())
}

pub fn define_environment(target: &str) {
  match target {
    "arm-unknown-linux-gnueabi" | "armv7-unknown-linux-gnueabi" => {
      println!("cargo:rustc-cfg=native")
    }
    _ => println!("cargo:rustc-cfg=simulation"),
  }
}
