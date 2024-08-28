pub mod deploy;
pub mod logging;
pub mod ssh;

use clap::{Parser, Subcommand};
use log::{error, info, warn};

use crate::deploy::DeployCommand;

pub struct Error {
  headline: String,
  specifics: Option<String>,
  remediation: Option<String>,
}

impl Error {
  pub fn new<I: Into<String>>(headline: I) -> Self {
    Self {
      headline: headline.into(),
      specifics: None,
      remediation: None,
    }
  }

  pub fn with_specifics<I: Into<String>>(self, specs: I) -> Self {
    Self {
      headline: self.headline,
      specifics: Some(specs.into()),
      remediation: self.remediation,
    }
  }

  pub fn with_remediation<I: Into<String>>(self, remediation: I) -> Self {
    Self {
      headline: self.headline,
      specifics: self.specifics,
      remediation: Some(remediation.into()),
    }
  }

  pub fn print(&self) {
    error!("{}", self.headline);
    if let Some(specifics) = &self.specifics {
      error!("{}", specifics);
    }
    if let Some(remediation) = &self.remediation {
      warn!("{}", remediation);
    }
  }
}

pub type CargoRobotResult<T> = std::result::Result<T, Error>;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
  #[command(subcommand)]
  command: Commands,
}

#[derive(Subcommand, Debug)]
enum Commands {
  Robot {
    #[command(subcommand)]
    command: RobotCommands,
  },
}

#[derive(Subcommand, Debug)]
enum RobotCommands {
  /// Deploy the robot program to the RoboRIO
  Deploy(deploy::DeployArgs),

  Prepare(deploy::DeployArgs),
}

fn main() {
  logging::init();
  let cli = Args::parse();

  let result = match cli.command {
    Commands::Robot { command } => match command {
      RobotCommands::Deploy(deploy_args) => {
        info!("Starting Deploy Command");

        DeployCommand::invoke(deploy_args, false)
      }
      RobotCommands::Prepare(deploy_args) => DeployCommand::invoke(deploy_args, true),
    },
  };

  match result {
    Ok(_) => info!("Completed Successfully!"),
    Err(e) => e.print(),
  }
}
