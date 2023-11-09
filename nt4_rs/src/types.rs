use std::{error::Error, fmt::Display, slice};

use wpilib_hal::{NT_Now, NT_String, NT_Type, NT_Value};

#[derive(Debug, Clone)]
pub enum NTError {
    TypeMismatch,
}

impl Display for NTError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            NTError::TypeMismatch => write!(f, "Type Mismatch"),
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

#[derive(Debug, Clone)]
pub enum Value {
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

#[derive(Debug, Clone)]
pub struct ValueContext {
    pub data: Value,
    pub last_change: Option<usize>,
    pub server_time: usize,
}

impl From<NT_Value> for ValueContext {
    fn from(value: NT_Value) -> Self {
        let data = match value.type_ {
            NT_Type::NT_UNASSIGNED => Value::Unassigned,
            NT_Type::NT_BOOLEAN => Value::Boolean(unsafe { value.data.v_boolean != 0 }),
            NT_Type::NT_DOUBLE => Value::Double(unsafe { value.data.v_double }),
            NT_Type::NT_STRING => Value::String({
                std::str::from_utf8(unsafe {
                    std::slice::from_raw_parts(
                        value.data.v_string.str_ as *const u8,
                        value.data.v_string.len as usize,
                    )
                })
                .unwrap()
                .to_owned()
            }),
            NT_Type::NT_RAW => Value::Raw({
                unsafe {
                    slice::from_raw_parts(value.data.v_raw.data, value.data.v_raw.size as usize)
                        .into()
                }
            }),
            NT_Type::NT_BOOLEAN_ARRAY => Value::BooleanArray({
                unsafe {
                    slice::from_raw_parts(
                        value.data.arr_boolean.arr,
                        value.data.arr_boolean.size as usize,
                    )
                    .iter()
                    .map(|x| *x != 0)
                    .collect()
                }
            }),
            NT_Type::NT_DOUBLE_ARRAY => Value::DoubleArray({
                unsafe {
                    slice::from_raw_parts(
                        value.data.arr_double.arr,
                        value.data.arr_double.size as usize,
                    )
                    .into()
                }
            }),
            NT_Type::NT_STRING_ARRAY => {
                let s_arr = unsafe {
                    slice::from_raw_parts(
                        value.data.arr_string.arr,
                        value.data.arr_string.size as usize,
                    )
                };
                Value::StringArray(
                    s_arr
                        .iter()
                        .map(|s| {
                            std::str::from_utf8(unsafe {
                                std::slice::from_raw_parts(s.str_ as *const u8, s.len as usize)
                            })
                            .unwrap()
                            .to_owned()
                        })
                        .collect(),
                )
            }
            NT_Type::NT_RPC => panic!("Cannot read an RPC"),
            NT_Type::NT_INTEGER => Value::Integer(unsafe { value.data.v_int as isize }),
            NT_Type::NT_FLOAT => Value::Float(unsafe { value.data.v_float }),
            NT_Type::NT_INTEGER_ARRAY => Value::IntegerArray({
                unsafe {
                    slice::from_raw_parts(value.data.arr_int.arr, value.data.arr_int.size as usize)
                        .iter()
                        .map(|x| *x as isize)
                        .collect()
                }
            }),
            NT_Type::NT_FLOAT_ARRAY => Value::FloatArray({
                unsafe {
                    slice::from_raw_parts(
                        value.data.arr_float.arr,
                        value.data.arr_float.size as usize,
                    )
                    .into()
                }
            }),
        };

        let last_change = match value.last_change {
            i if i < 0 => None,
            i => Some(i as usize),
        };

        ValueContext {
            data,
            last_change,
            server_time: value.server_time as usize,
        }
    }
}

impl Value {
    pub fn with_nt_value<T, F: FnOnce(NT_Value) -> T>(&self, f: F) -> T {
        let last_change = unsafe { NT_Now() };
        let server_time = unsafe { NT_Now() };

        let mut ntv = NT_Value::default();
        ntv.last_change = last_change;
        ntv.server_time = server_time;

        match self {
            Value::Unassigned => f(NT_Value {
                type_: NT_Type::NT_UNASSIGNED,
                ..ntv
            }),
            Value::Boolean(v) => f({
                ntv.type_ = NT_Type::NT_BOOLEAN;
                ntv.data.v_boolean = *v as i32;
                ntv
            }),
            Value::Double(v) => f({
                ntv.type_ = NT_Type::NT_DOUBLE;
                ntv.data.v_double = *v;
                ntv
            }),
            Value::String(v) => {
                ntv.type_ = NT_Type::NT_STRING;
                ntv.data.v_string.len = v.as_bytes().len();
                ntv.data.v_string.str_ = v.as_ptr() as *mut u8; // These casts are very unsafe, but we make the assumption that NT doesn't mutate the pointer
                f(ntv)
            }
            Value::Raw(v) => {
                ntv.type_ = NT_Type::NT_RAW;
                ntv.data.v_raw.size = v.len();
                ntv.data.v_raw.data = v.as_ptr() as *mut u8; // These casts are very unsafe, but we make the assumption that NT doesn't mutate the pointer
                f(ntv)
            }
            Value::BooleanArray(arr) => {
                let mut buf = vec![0; arr.len()];
                for i in 0..arr.len() {
                    buf[i] = arr[i] as i32
                }

                ntv.type_ = NT_Type::NT_BOOLEAN_ARRAY;
                ntv.data.arr_boolean.size = arr.len();
                ntv.data.arr_boolean.arr = buf.as_ptr() as *mut i32;
                f(ntv)
            }
            Value::DoubleArray(arr) => {
                ntv.type_ = NT_Type::NT_DOUBLE_ARRAY;
                ntv.data.arr_double.size = arr.len();
                ntv.data.arr_double.arr = arr.as_ptr() as *mut f64;
                f(ntv)
            }
            Value::StringArray(arr) => {
                let mut buf = vec![Default::default(); arr.len()];
                for i in 0..arr.len() {
                    buf[i] = NT_String {
                        str_: arr[i].as_ptr() as *mut u8,
                        len: arr[i].len(),
                    }
                }

                ntv.type_ = NT_Type::NT_STRING_ARRAY;
                ntv.data.arr_string.size = arr.len();
                ntv.data.arr_string.arr = buf.as_ptr() as *mut NT_String;
                f(ntv)
            }
            Value::Integer(v) => f({
                ntv.type_ = NT_Type::NT_INTEGER;
                ntv.data.v_int = *v as i64;
                ntv
            }),
            Value::Float(v) => f({
                ntv.type_ = NT_Type::NT_FLOAT;
                ntv.data.v_float = *v;
                ntv
            }),
            Value::IntegerArray(arr) => {
                let mut buf = vec![0; arr.len()];
                for i in 0..arr.len() {
                    buf[i] = arr[i] as i64
                }

                ntv.type_ = NT_Type::NT_INTEGER_ARRAY;
                ntv.data.arr_int.size = arr.len();
                ntv.data.arr_int.arr = buf.as_ptr() as *mut i64;
                f(ntv)
            }
            Value::FloatArray(arr) => {
                ntv.type_ = NT_Type::NT_FLOAT_ARRAY;
                ntv.data.arr_float.size = arr.len();
                ntv.data.arr_float.arr = arr.as_ptr() as *mut f32;
                f(ntv)
            }
        }
    }
}

macro_rules! value_from {
    ($variant:ident, $ty:ty) => {
        impl From<$ty> for Value {
            fn from(value: $ty) -> Self {
                Value::$variant(value)
            }
        }
    };
}

value_from!(Boolean, bool);
value_from!(Double, f64);
value_from!(String, String);
value_from!(BooleanArray, Vec<bool>);
value_from!(DoubleArray, Vec<f64>);
value_from!(StringArray, Vec<String>);
value_from!(Integer, isize);
value_from!(Float, f32);
value_from!(IntegerArray, Vec<isize>);
value_from!(FloatArray, Vec<f32>);
