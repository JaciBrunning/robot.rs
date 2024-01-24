use std::{process::Stdio, path::PathBuf, net::{SocketAddr, ToSocketAddrs}};

use cargo_metadata::{MetadataCommand, Metadata, Package};
use clap::Args;
use log::info;
use robot_rs_build_utils::{maybe_download_libs, Profile};

use crate::{CargoRobotResult, Error, ssh::SSHSession};

#[derive(Debug, Args)]
pub struct DeployArgs {
  /// Deploy with release mode builds enabled
  #[arg(short, long)]
  release: bool,

  /// Deploy an example
  #[arg(long)]
  example: Option<String>,

  /// Deploy a specific binary
  #[arg(long)]
  bin: Option<String>,

  #[arg(long)]
  no_default_features: bool,

  #[arg(long)]
  features: Option<String>,

  /// Deploy with a specific RoboRIO Team Number
  #[arg(short, long)]
  team: Option<usize>,

  /// Deploy with a specific RoboRIO IP Address
  #[arg(long)]
  ip: Option<String>,

  /// Path to the project, if it's not in the current directory
  #[arg(long)]
  project: Option<String>
}

pub struct DeployCommand;

impl DeployCommand {
  pub fn invoke(args: DeployArgs, dry_run: bool) -> CargoRobotResult<()> {
    let mut meta = MetadataCommand::new();
    if let Some(project_dir) = &args.project {
      meta.current_dir(project_dir);
    }
    let meta = meta.exec().map_err(|_| Error::new("Failed to get Metadata").with_remediation("Are you in the correct directory? If you're using --project, is it correct?"))?;

    Self::cargo_build(&args)?;
    let libs = Self::collect_libs_for_deploy(&meta, if args.release { Profile::Release } else { Profile::Debug })?;

    if !dry_run {
      let package = meta.root_package().ok_or(Error::new("You can't run deploy in a workspace!").with_remediation("Either cd into a directory, or use --project"))?;
      let ips = match args.ip {
        Some(ip) => vec![ip],
        None => match Self::get_robot_metadata_entry(package, "ip") {
          Ok(ips) => ips.as_str().unwrap().split(",").map(ToOwned::to_owned).collect(),
          Err(_) => match args.team {
            Some(team) => Self::generate_ips_for_team(team),
            None => match Self::get_robot_metadata_entry(package, "team") {
              Ok(team) => Self::generate_ips_for_team(team.as_u64().unwrap() as usize),
              Err(e) => Err(e)?
            }
          }
        }
      };

      let mut errors = vec![];

      let path_to_artifact = meta.target_directory.join("arm-unknown-linux-gnueabi")
        .join(if args.release { "release" } else { "debug" })
        .join(match (args.example, args.bin) {
          (_, Some(bin)) => bin.clone(),
          (Some(ex), _) => format!("examples/{}", ex),
          _ => package.name.clone()
        });

      info!("Collected {} native libraries for deploy", libs.len());
      for ip in ips {
        match Self::do_deploy(&ip, &libs, path_to_artifact.clone().into()) {
          Ok(_) => return Ok(()),
          Err(e) => errors.push(e)
        }
      }

      for error in errors {
        error.print()
      }

      Err(Error::new("Could not connect to target.").with_remediation("Is your Robot on and connected to your computer?"))
    } else {
      Ok(())
    }
  }

  fn do_deploy(ip: &str, libs: &Vec<PathBuf>, path_to_artifact: PathBuf) -> CargoRobotResult<()> {
    let addrs: Vec<_> = (ip, 22).to_socket_addrs().map_err(|e| Error::new(format!("Could not resolve hostname: {} - {}", ip, e)))?.collect();

    let mut errs = vec![];

    for addr in addrs {
      match Self::try_deploy(addr, libs, &path_to_artifact) {
        Ok(_) => return Ok(()),
        Err(e) => errs.push(format!("Addr: {} -> {}", addr, e.headline))
      }
    }

    Err(Error::new(format!("Deploy Failure: {}", ip))
    .with_specifics(errs.join("; ")))
  }

  fn try_deploy(addr: SocketAddr, libs: &Vec<PathBuf>, path_to_artifact: &PathBuf) -> CargoRobotResult<()> {
    // Deploy libs first
    let session = SSHSession::connect(addr, "admin", "").map_err(|e| Error::new(format!("Could not establish an SSH connection: {}", e)))?;
    for lib in libs {
      let target = PathBuf::from(format!("/usr/local/frc/third-party/lib/{}", lib.file_name().unwrap().to_str().unwrap()));
      session.maybe_copy_file(lib, target.as_path(), 0o755, true)
        .map_err(|e| Error::new(format!("Could not copy file {} -> {}: {}", lib.file_name().unwrap().to_str().unwrap(), target.to_str().unwrap(), e)))?;
    }

    // Deploy our program
    session.run(". /etc/profile.d/natinst-path.sh; /usr/local/frc/bin/frcKillRobot.sh -t 2> /dev/null").ok();

    session.maybe_copy_file(path_to_artifact, PathBuf::from(format!("/home/lvuser/frcUserProgram")).as_path(), 0o755, false)
      .map_err(|e| Error::new(format!("Could not copy user program")).with_specifics(format!("{}", e)))?;
    
    session.run("chmod +x /home/lvuser/frcUserProgram && chown lvuser /home/lvuser/frcUserProgram")
      .map_err(|e| Error::new("Could not chmod user program").with_specifics(format!("{}", e)))?;
    session.run("setcap cap_sys_nice+eip /home/lvuser/frcUserProgram")
      .map_err(|e| Error::new("Could not set CAP_SYS_NICE").with_specifics(format!("{}", e)))?;

    let robot_command = "/home/lvuser/frcUserProgram";

    session.run(&format!("echo \"{}\" > /home/lvuser/robotCommand", robot_command))
      .map_err(|e| Error::new("Could not write robotCommand").with_specifics(format!("{}", e)))?;

    session.run("chmod +x /home/lvuser/robotCommand && chown lvuser /home/lvuser/robotCommand")
      .map_err(|e| Error::new("Could not chmod robotCommand").with_specifics(format!("{}", e)))?;

    session.run(". /etc/profile.d/natinst-path.sh; /usr/local/frc/bin/frcKillRobot.sh -t -r 2> /dev/null").ok();
    session.run("ldconfig").ok();

    Ok(())
  }

  fn generate_ips_for_team(team: usize) -> Vec<String> {
    vec![
      "172.22.11.2".to_owned(),
      format!("10.{}.{}.2", team / 100, team % 100),
      format!("roborio-{}-FRC.local", team),
      format!("roborio-{}-FRC.frc-field.local", team),
    ]
  }

  fn get_robot_metadata_entry(package: &Package, key: &str) -> CargoRobotResult<serde_json::Value> {
    match package.metadata.get("robot") {
      Some(robot) => match robot.get(key) {
        Some(key) => Ok(key.clone()),
        None => Err(Error::new("Missing '{}' key in [metadata.robot]").with_remediation("Add '{} = <value>' below your [metadata.robot] entry in your Cargo.toml"))
      },
      None => Err(Error::new("Missing [metadata.robot]").with_remediation("Add [metadata.robot] to the end of your Cargo.toml")),
    }
  }

  fn collect_libs_for_deploy(meta: &Metadata, profile: Profile) -> CargoRobotResult<Vec<PathBuf>> {
    let mut library_paths = vec![];

    for package in &meta.packages {
      if let Some(meta) = package.metadata.as_object() {
        let (_header_dirs, _link_dirs, runtime_dirs) = maybe_download_libs(package, "arm-unknown-linux-gnueabi", profile)
          .map_err(|e| Error::new("Failed to Download and Extract Dependency Libs").with_remediation("Try running `cargo robot prepare` to download files ahead of time").with_specifics(format!("Error: {}", e)))?;

        for dir in runtime_dirs {
          for path in std::fs::read_dir(&dir).map_err(|e| Error::new("Internal error: failed to read runtime path.").with_specifics(format!("{}", e)))? {
            if let Ok(path) = path {
              library_paths.push(path.path());
            }
          }
        }
      }
    }

    Ok(library_paths)
  }

  fn cargo_build(args: &DeployArgs) -> CargoRobotResult<()> {
    let cargo = std::env::var("CARGO").unwrap_or("cargo".to_owned());
    let mut cmd = std::process::Command::new(cargo);
    if let Some(project_dir) = &args.project {
      cmd.current_dir(project_dir);
    }
    cmd.args(["build", "--target=arm-unknown-linux-gnueabi"]);
    if args.no_default_features {
      cmd.arg("--no-default-features");
    }

    if let Some(features) = &args.features {
      cmd.arg("--features=".to_owned() + features);
    }

    if args.release {
      cmd.arg("--release");
    }

    if let Some(ex) = &args.example {
      cmd.arg("--example").arg(ex);
    }
    
    if let Some(bin) = &args.bin {
      cmd.arg("--bin").arg(bin);
    }

    cmd.stderr(Stdio::inherit());

    let output = cmd.output().map_err(|e| Error::new("Failed to launch command").with_specifics(format!("{}", e)))?;
    if !output.status.success() {
      return Err(Error::new("Failed to build!"))
    }
    Ok(())
  }
}