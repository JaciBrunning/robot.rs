use ntcore_rs_macros::NTStruct;
use ntcore_rs::nt_structs::NTStruct;
use ntcore_rs::nt_structs::BytesMut;

#[derive(Debug, NTStruct, PartialEq)]
#[nt(type_string_fragment = "Translation2d")]
pub struct Translation2d {
  pub x: f64,
  pub y: f64,
}

#[derive(Debug, NTStruct, PartialEq)]
#[nt(type_string_fragment = "Rotation2d")]
pub struct Rotation2d {
  pub value: f64
}

#[derive(Debug, NTStruct, PartialEq)]
#[nt(type_string_fragment = "Transform2d")]
pub struct Transform2d {
  pub translation: Translation2d,
  pub rotation: Rotation2d
}

#[test]
fn main() {
  assert_eq!(Transform2d::TYPE_STRING_FRAG, "Transform2d");
  assert_eq!(Transform2d::get_schema(), "Translation2d translation;Rotation2d rotation");

  let tform = Transform2d {
    translation: Translation2d { x: 12.34, y: -14.23 },
    rotation: Rotation2d { value: 2.14 }
  };

  let mut buf = BytesMut::with_capacity(128);
  tform.write(&mut buf).unwrap();

  let new_tform = Transform2d::read(&mut buf).unwrap();
  assert_eq!(tform, new_tform);
}