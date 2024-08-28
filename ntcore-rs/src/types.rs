use std::{error::Error, ffi::CString, fmt::Display, slice};

use robot_rs_ntcore_sys::NT_DisposeValue;

use crate::nt_internal::{NT_Now, NT_String, NT_Type, NT_Value};

#[derive(Debug)]
pub enum NTError {
  TypeMismatch,
  Other(Box<dyn Error>),
}

impl Display for NTError {
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    match self {
      NTError::TypeMismatch => write!(f, "Type Mismatch"),
      NTError::Other(other) => other.fmt(f),
    }
  }
}
impl Error for NTError {}

pub type NTResult<T> = Result<T, NTError>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Type {
  Unassigned,
  Boolean,
  Double,
  String,
  Raw,
  BooleanArray,
  DoubleArray,
  StringArray,
  Rpc,
  Integer,
  Float,
  IntegerArray,
  FloatArray,
}

impl From<NT_Type> for Type {
  fn from(value: NT_Type) -> Self {
    match value {
      NT_Type::NT_UNASSIGNED => Self::Unassigned,
      NT_Type::NT_BOOLEAN => Self::Boolean,
      NT_Type::NT_DOUBLE => Self::Double,
      NT_Type::NT_STRING => Self::String,
      NT_Type::NT_RAW => Self::Raw,
      NT_Type::NT_BOOLEAN_ARRAY => Self::BooleanArray,
      NT_Type::NT_DOUBLE_ARRAY => Self::DoubleArray,
      NT_Type::NT_STRING_ARRAY => Self::StringArray,
      NT_Type::NT_RPC => Self::Rpc,
      NT_Type::NT_INTEGER => Self::Integer,
      NT_Type::NT_FLOAT => Self::Float,
      NT_Type::NT_INTEGER_ARRAY => Self::IntegerArray,
      NT_Type::NT_FLOAT_ARRAY => Self::FloatArray,
    }
  }
}

impl From<Type> for NT_Type {
  fn from(val: Type) -> Self {
    match val {
      Type::Unassigned => NT_Type::NT_UNASSIGNED,
      Type::Boolean => NT_Type::NT_BOOLEAN,
      Type::Double => NT_Type::NT_DOUBLE,
      Type::String => NT_Type::NT_STRING,
      Type::Raw => NT_Type::NT_RAW,
      Type::BooleanArray => NT_Type::NT_BOOLEAN_ARRAY,
      Type::DoubleArray => NT_Type::NT_DOUBLE_ARRAY,
      Type::StringArray => NT_Type::NT_STRING_ARRAY,
      Type::Rpc => NT_Type::NT_RPC,
      Type::Integer => NT_Type::NT_INTEGER,
      Type::Float => NT_Type::NT_FLOAT,
      Type::IntegerArray => NT_Type::NT_INTEGER_ARRAY,
      Type::FloatArray => NT_Type::NT_FLOAT_ARRAY,
    }
  }
}

#[derive(Debug, Clone)]
pub enum NTValue {
  Unassigned,
  Boolean(bool),
  Double(f64),
  String(String),
  Raw(Vec<u8>),
  BooleanArray(Vec<bool>),
  DoubleArray(Vec<f64>),
  StringArray(Vec<String>),
  Integer(isize),
  Float(f32),
  IntegerArray(Vec<isize>),
  FloatArray(Vec<f32>),
}

impl From<NT_Value> for NTValue {
  fn from(mut value: NT_Value) -> Self {
    let data = match value.type_ {
      NT_Type::NT_UNASSIGNED => NTValue::Unassigned,
      NT_Type::NT_BOOLEAN => NTValue::Boolean(unsafe { value.data.v_boolean != 0 }),
      NT_Type::NT_DOUBLE => NTValue::Double(unsafe { value.data.v_double }),
      NT_Type::NT_STRING => NTValue::String({
        std::str::from_utf8(unsafe {
          std::slice::from_raw_parts(
            value.data.v_string.str_ as *const u8,
            value.data.v_string.len,
          )
        })
        .unwrap()
        .to_owned()
      }),
      NT_Type::NT_RAW => NTValue::Raw({
        unsafe { slice::from_raw_parts(value.data.v_raw.data, value.data.v_raw.size).into() }
      }),
      NT_Type::NT_BOOLEAN_ARRAY => NTValue::BooleanArray({
        unsafe {
          slice::from_raw_parts(value.data.arr_boolean.arr, value.data.arr_boolean.size)
            .iter()
            .map(|x| *x != 0)
            .collect()
        }
      }),
      NT_Type::NT_DOUBLE_ARRAY => NTValue::DoubleArray({
        unsafe {
          slice::from_raw_parts(value.data.arr_double.arr, value.data.arr_double.size).into()
        }
      }),
      NT_Type::NT_STRING_ARRAY => {
        let s_arr =
          unsafe { slice::from_raw_parts(value.data.arr_string.arr, value.data.arr_string.size) };
        NTValue::StringArray(
          s_arr
            .iter()
            .map(|s| {
              std::str::from_utf8(unsafe { std::slice::from_raw_parts(s.str_ as *const u8, s.len) })
                .unwrap()
                .to_owned()
            })
            .collect(),
        )
      }
      NT_Type::NT_RPC => panic!("Cannot read an RPC"),
      NT_Type::NT_INTEGER => NTValue::Integer(unsafe { value.data.v_int as isize }),
      NT_Type::NT_FLOAT => NTValue::Float(unsafe { value.data.v_float }),
      NT_Type::NT_INTEGER_ARRAY => NTValue::IntegerArray({
        unsafe {
          slice::from_raw_parts(value.data.arr_int.arr, value.data.arr_int.size as usize)
            .iter()
            .map(|x| *x as isize)
            .collect()
        }
      }),
      NT_Type::NT_FLOAT_ARRAY => NTValue::FloatArray({
        unsafe {
          slice::from_raw_parts(value.data.arr_float.arr, value.data.arr_float.size as usize).into()
        }
      }),
    };

    unsafe { NT_DisposeValue((&mut value) as *mut NT_Value) };

    data
  }
}

impl NTValue {
  pub fn with_nt_value<T, F: FnOnce(NT_Value) -> T>(&self, f: F) -> T {
    let last_change = unsafe { NT_Now() };
    let server_time = unsafe { NT_Now() };

    let mut ntv = NT_Value::default();
    ntv.last_change = last_change;
    ntv.server_time = server_time;

    match self {
      NTValue::Unassigned => f(NT_Value {
        type_: NT_Type::NT_UNASSIGNED,
        ..ntv
      }),
      NTValue::Boolean(v) => f({
        ntv.type_ = NT_Type::NT_BOOLEAN;
        ntv.data.v_boolean = *v as i32;
        ntv
      }),
      NTValue::Double(v) => f({
        ntv.type_ = NT_Type::NT_DOUBLE;
        ntv.data.v_double = *v;
        ntv
      }),
      NTValue::String(v) => {
        let cstr = CString::new(v.clone()).unwrap();
        ntv.type_ = NT_Type::NT_STRING;
        ntv.data.v_string.len = v.as_bytes().len();
        ntv.data.v_string.str_ = cstr.as_ptr() as *mut _; // These casts are very unsafe, but we make the assumption that NT doesn't mutate the pointer
        f(ntv)
      }
      NTValue::Raw(v) => {
        ntv.type_ = NT_Type::NT_RAW;
        ntv.data.v_raw.size = v.len();
        ntv.data.v_raw.data = v.as_ptr() as *mut u8; // These casts are very unsafe, but we make the assumption that NT doesn't mutate the pointer
        f(ntv)
      }
      NTValue::BooleanArray(arr) => {
        let mut buf = vec![0; arr.len()];
        for i in 0..arr.len() {
          buf[i] = arr[i] as i32
        }

        ntv.type_ = NT_Type::NT_BOOLEAN_ARRAY;
        ntv.data.arr_boolean.size = arr.len();
        ntv.data.arr_boolean.arr = buf.as_ptr() as *mut i32;
        f(ntv)
      }
      NTValue::DoubleArray(arr) => {
        ntv.type_ = NT_Type::NT_DOUBLE_ARRAY;
        ntv.data.arr_double.size = arr.len();
        ntv.data.arr_double.arr = arr.as_ptr() as *mut f64;
        f(ntv)
      }
      NTValue::StringArray(arr) => {
        let mut buf = vec![Default::default(); arr.len()];
        for i in 0..arr.len() {
          buf[i] = NT_String {
            str_: arr[i].as_ptr() as *mut _,
            len: arr[i].len(),
          }
        }

        ntv.type_ = NT_Type::NT_STRING_ARRAY;
        ntv.data.arr_string.size = arr.len();
        ntv.data.arr_string.arr = buf.as_ptr() as *mut NT_String;
        f(ntv)
      }
      NTValue::Integer(v) => f({
        ntv.type_ = NT_Type::NT_INTEGER;
        ntv.data.v_int = *v as i64;
        ntv
      }),
      NTValue::Float(v) => f({
        ntv.type_ = NT_Type::NT_FLOAT;
        ntv.data.v_float = *v;
        ntv
      }),
      NTValue::IntegerArray(arr) => {
        let mut buf = vec![0; arr.len()];
        for i in 0..arr.len() {
          buf[i] = arr[i] as i64
        }

        ntv.type_ = NT_Type::NT_INTEGER_ARRAY;
        ntv.data.arr_int.size = arr.len();
        ntv.data.arr_int.arr = buf.as_ptr() as *mut i64;
        f(ntv)
      }
      NTValue::FloatArray(arr) => {
        ntv.type_ = NT_Type::NT_FLOAT_ARRAY;
        ntv.data.arr_float.size = arr.len();
        ntv.data.arr_float.arr = arr.as_ptr() as *mut f32;
        f(ntv)
      }
    }
  }
}

macro_rules! simple_value_from {
  ($variant:ident, $ty:ty, $nt_type_str:literal) => {
    impl Value for $ty {
      const NT_TYPE_STRING: &'static str = $nt_type_str;
      const NT_TYPE: Type = Type::$variant;

      fn from_nt(val: NTValue) -> NTResult<Self> {
        match val {
          NTValue::$variant(v) => Ok(v),
          _ => Err(NTError::TypeMismatch),
        }
      }

      fn to_nt(self) -> NTValue {
        NTValue::$variant(self)
      }
    }
  };
}

pub trait Value: Sized {
  const NT_TYPE_STRING: &'static str;
  const NT_TYPE: Type;

  fn from_nt(val: NTValue) -> NTResult<Self>;
  fn to_nt(self) -> NTValue;
}

simple_value_from!(Boolean, bool, "boolean");
simple_value_from!(Float, f32, "float");
simple_value_from!(Double, f64, "double");
simple_value_from!(String, String, "string");
simple_value_from!(Raw, Vec<u8>, "raw");
simple_value_from!(BooleanArray, Vec<bool>, "boolean[]");
simple_value_from!(FloatArray, Vec<f32>, "float[]");
simple_value_from!(DoubleArray, Vec<f64>, "double[]");
simple_value_from!(StringArray, Vec<String>, "string[]");
simple_value_from!(Integer, isize, "int");
simple_value_from!(IntegerArray, Vec<isize>, "int[]");
