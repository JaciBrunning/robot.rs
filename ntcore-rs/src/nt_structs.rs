use std::{error::Error, io::Read};

use bytes::BufMut;
pub use bytes::BytesMut;

pub trait NTStruct: Sized {
  const TYPE_STRING_FRAG: &'static str;
  fn get_full_type_string() -> String {
    format!("struct:{}", Self::TYPE_STRING_FRAG)
  }

  fn get_schema() -> String;
  fn publish_schema(inst: u32);
  fn write(&self, buf: &mut BytesMut) -> Result<(), Box<dyn Error>>;
  fn read(buf: &mut BytesMut) -> Result<Self, Box<dyn Error>>;
}

impl NTStruct for bool {
  const TYPE_STRING_FRAG: &'static str = "bool";

  fn get_schema() -> String {
    "bool value".to_owned()
  }

  fn publish_schema(_inst: u32) {}

  fn write(&self, buf: &mut BytesMut) -> Result<(), Box<dyn Error>> {
    buf.put(&[if *self { 1u8 } else { 0u8 }][..]);
    Ok(())
  }

  fn read(buf: &mut BytesMut) -> Result<Self, Box<dyn Error>> {
    let mut b = [0u8; 1];
    buf.take(b.len() as u64).read_exact(&mut b)?;
    Ok(if b[0] > 0 { true } else { false })
  }
}

macro_rules! nt_struct_int_t {
  ($ty:ty, $type_str:literal, $schema:literal) => {
    impl NTStruct for $ty {
      const TYPE_STRING_FRAG: &'static str = $type_str;

      fn get_schema() -> String {
        $schema.to_owned()
      }

      fn publish_schema(_inst: u32) {}

      fn write(&self, buf: &mut BytesMut) -> Result<(), Box<dyn Error>> {
        buf.put(&self.to_le_bytes()[..]);
        Ok(())
      }

      fn read(buf: &mut BytesMut) -> Result<Self, Box<dyn Error>> {
        let mut b = [0u8; Self::BITS as usize / 8];
        let new = buf.split_to(b.len());
        b.copy_from_slice(&new);
        Ok(Self::from_le_bytes(b))
      }
    }
  };
}

nt_struct_int_t!(u8, "uint8", "uint8 value");
nt_struct_int_t!(i8, "int8", "int8 value");
nt_struct_int_t!(u16, "uint16", "uint16 value");
nt_struct_int_t!(i16, "int16", "int16 value");
nt_struct_int_t!(u32, "uint32", "uint32 value");
nt_struct_int_t!(i32, "int32", "int32 value");
nt_struct_int_t!(u64, "uint64", "uint64 value");
nt_struct_int_t!(i64, "int64", "int64 value");

impl NTStruct for f32 {
  const TYPE_STRING_FRAG: &'static str = "float";

  fn get_schema() -> String {
    "float value".to_owned()
  }

  fn publish_schema(_inst: u32) {
    todo!()
  }

  fn write(&self, buf: &mut BytesMut) -> Result<(), Box<dyn Error>> {
    let as_u32 = unsafe { std::mem::transmute::<f32, u32>(*self) };
    as_u32.write(buf)
  }

  fn read(buf: &mut BytesMut) -> Result<Self, Box<dyn Error>> {
    let as_u32 = u32::read(buf)?;
    Ok(unsafe { std::mem::transmute::<u32, f32>(as_u32) })
  }
}

impl NTStruct for f64 {
  const TYPE_STRING_FRAG: &'static str = "double";

  fn get_schema() -> String {
    "double value".to_owned()
  }

  fn publish_schema(_inst: u32) {}

  fn write(&self, buf: &mut BytesMut) -> Result<(), Box<dyn Error>> {
    let as_u64 = unsafe { std::mem::transmute::<f64, u64>(*self) };
    as_u64.write(buf)
  }

  fn read(buf: &mut BytesMut) -> Result<Self, Box<dyn Error>> {
    let as_u64 = u64::read(buf)?;
    Ok(unsafe { std::mem::transmute::<u64, f64>(as_u64) })
  }
}
