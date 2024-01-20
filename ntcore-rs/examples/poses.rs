use std::time::Duration;

use ntcore_rs::GenericPublisher;
use ntcore_rs::ServerConfig;
use ntcore_rs_macros::NTStruct;
use ntcore_rs::nt_structs::NTStruct;
use ntcore_rs::nt_structs::BytesMut;

#[derive(Debug, Clone, NTStruct, PartialEq)]
#[nt(type_string_fragment = "Translation2d")]
pub struct Translation2d {
  pub x: f64,
  pub y: f64,
}

#[derive(Debug, Clone, NTStruct, PartialEq)]
#[nt(type_string_fragment = "Rotation2d")]
pub struct Rotation2d {
  pub value: f64
}

#[derive(Debug, Clone, NTStruct, PartialEq)]
#[nt(type_string_fragment = "Pose2d")]
pub struct Pose2d {
  pub translation: Translation2d,
  pub rotation: Rotation2d
}

fn main() {
  let tform = Pose2d {
    translation: Translation2d { x: 0.0, y: 0.0 },
    rotation: Rotation2d { value: 0.0 }
  };

  let mut nt = ntcore_rs::NetworkTableInstance::default();
  nt.start_server(ServerConfig::default());

  let topic = nt.topic("/test/pose");
  let publish = topic.publish_struct();

  loop {
    publish.set(tform.clone()).unwrap();
    std::thread::sleep(Duration::from_millis(1000));
  }
}