use std::ffi::CString;
use std::marker::PhantomData;
use std::mem::size_of;

use bytes::BytesMut;
use robot_rs_ntcore_sys::{NT_Subscriber, NT_Unsubscribe, NT_Subscribe, NT_PubSubOptions, NT_Publish, NT_Unpublish, NT_Publisher, NT_GetEntryEx, NT_GetEntryValue, NT_Value, NT_Entry, NT_SetEntryValue, NT_Inst, NT_PublishEx, NT_AddSchema};

use crate::NTValue;
use crate::nt_internal::{NT_Topic, NT_GetTopic, NT_GetTopicType, NT_GetTopicTypeString, NT_SetTopicPersistent, NT_GetTopicPersistent, NT_SetTopicRetained, NT_GetTopicRetained, NT_GetTopicExists};

use crate::nt_structs::NTStruct;
use crate::types::{Value, NTError, NTResult};
use crate::{instance::NetworkTableInstance, types::Type};

const DEFAULT_PUBSUB: NT_PubSubOptions = NT_PubSubOptions {
  structSize: size_of::<NT_PubSubOptions>() as u32,
  pollStorage: 0,
  periodic: 0.1,
  excludePublisher: 0,
  sendAll: 0,
  topicsOnly: 0,
  prefixMatch: 0,
  keepDuplicates: 0,
  disableRemote: 0,
  disableLocal: 0,
  excludeSelf: 0,
};

pub trait GenericSubscriber<V> {
  fn get(&self) -> Option<NTResult<V>>;
}

pub trait GenericPublisher<V> {
  fn set(&self, v: V) -> NTResult<()>;
}

pub struct Subscriber<V: Value> {
  handle: NT_Subscriber,
  value_t: PhantomData<V>
}

impl<V: Value> GenericSubscriber<V> for Subscriber<V> {
  fn get(&self) -> Option<NTResult<V>> {
    let mut value = NT_Value::default();
    unsafe { NT_GetEntryValue(self.handle, &mut value) }
    match value.type_ {
      robot_rs_ntcore_sys::NT_Type::NT_UNASSIGNED => None,
      _ => Some(V::from_nt(value.into()))
    }
  }
}

impl<V: Value> Drop for Subscriber<V> {
  fn drop(&mut self) {
    unsafe { NT_Unsubscribe(self.handle) }
  }
}

pub struct Publisher<V: Value> {
  handle: NT_Publisher,
  value_t: PhantomData<V>
}

impl<V: Value> GenericPublisher<V> for Publisher<V> {
  fn set(&self, v: V) -> NTResult<()> {
    v.to_nt().with_nt_value(|ntval| {
      if (unsafe { NT_SetEntryValue(self.handle, &ntval) }) == 0 {
        Err(NTError::TypeMismatch)
      } else {
        Ok(())
      }
    })
  }
}

impl<V: Value> Drop for Publisher<V> {
  fn drop(&mut self) {
    unsafe { NT_Unpublish(self.handle) }
  }
}

pub struct Entry<V: Value> {
  handle: NT_Entry,
  value_t: PhantomData<V>
}

impl<V: Value> GenericPublisher<V> for Entry<V> {
  fn set(&self, v: V) -> NTResult<()> {
    v.to_nt().with_nt_value(|ntval| {
      if (unsafe { NT_SetEntryValue(self.handle, &ntval) }) == 0 {
        Err(NTError::TypeMismatch)
      } else {
        Ok(())
      }
    })
  }
}

impl<V: Value> GenericSubscriber<V> for Entry<V> {
  fn get(&self) -> Option<NTResult<V>> {
    let mut value = NT_Value::default();
    unsafe { NT_GetEntryValue(self.handle, &mut value) }
    match value.type_ {
      robot_rs_ntcore_sys::NT_Type::NT_UNASSIGNED => None,
      _ => Some(V::from_nt(value.into()))
    }
  }
}

impl<V: Value> Drop for Entry<V> {
  fn drop(&mut self) {
    unsafe { NT_Unpublish(self.handle) }
  }
}

pub struct StructSubscriber<T: NTStruct> {
  handle: NT_Subscriber,
  value_t: PhantomData<T>
}

impl<T: NTStruct> GenericSubscriber<T> for StructSubscriber<T> {
  fn get(&self) -> Option<NTResult<T>> {
    let mut value = NT_Value::default();
    unsafe { NT_GetEntryValue(self.handle, &mut value) }
    match value.type_ {
      robot_rs_ntcore_sys::NT_Type::NT_UNASSIGNED => None,
      _ => {
        let v = Vec::<u8>::from_nt(value.into());
        match v {
          Ok(v) => {
            let mut bytes = BytesMut::from(&v[..]);
            Some(T::read(&mut bytes).map_err(NTError::Other))
          },
          Err(e) => Some(Err(e)),
        }
      }
    }
  }
}

impl<T: NTStruct> Drop for StructSubscriber<T> {
  fn drop(&mut self) {
    unsafe { NT_Unsubscribe(self.handle) }
  }
}

pub struct StructPublisher<T: NTStruct> {
  handle: NT_Publisher,
  value_t: PhantomData<T>
}

impl<T: NTStruct> GenericPublisher<T> for StructPublisher<T> {
  fn set(&self, v: T) -> NTResult<()> {
    let mut bytes = BytesMut::with_capacity(128);
    v.write(&mut bytes).map_err(NTError::Other)?;
    let raw = NTValue::Raw(bytes.to_vec());
    raw.with_nt_value(|ntval| {
      if (unsafe { NT_SetEntryValue(self.handle, &ntval) }) == 0 {
        Err(NTError::TypeMismatch)
      } else {
        Ok(())
      }
    })?;
    Ok(())
  }
}

impl<T: NTStruct> Drop for StructPublisher<T> {
  fn drop(&mut self) {
    unsafe { NT_Unpublish(self.handle) }
  }
}

pub struct Topic {
  handle: NT_Topic,
  name: String,
  instance_handle: NT_Inst
}

impl Topic {
  pub fn new(instance: &NetworkTableInstance, name: &str) -> Self {
    Self::from_inst_handle(instance.handle, name)
  }

  fn from_inst_handle(handle: NT_Inst, name: &str) -> Self {
    let cstr = CString::new(name).unwrap();
    Self {
      instance_handle: handle,
      handle: unsafe {
        NT_GetTopic(handle, cstr.as_ptr(), cstr.as_bytes().len())
      },
      name: name.to_owned()
    }
  }

  pub fn child(&self, name: &str) -> Self {
    Self::from_inst_handle(self.instance_handle, &(self.name.clone() + "/" + name))
  }

  pub fn get_type(&self) -> Type {
    unsafe { NT_GetTopicType(self.handle) }.into()
  }

  pub fn get_type_str(&self) -> String {
    let mut len = 0;
    let buf = unsafe { NT_GetTopicTypeString(self.handle, &mut len) };
    std::str::from_utf8(unsafe { std::slice::from_raw_parts(buf as *const u8, len as usize) }).unwrap().to_owned()
  }

  pub fn set_persistent(&mut self, persistent: bool) {
    unsafe { NT_SetTopicPersistent(self.handle, persistent as i32) }
  }

  pub fn is_persistent(&self) -> bool {
    unsafe { NT_GetTopicPersistent(self.handle) != 0 }
  }

  pub fn set_retained(&mut self, retained: bool) {
    unsafe { NT_SetTopicRetained(self.handle, retained as i32) }
  }

  pub fn is_retained(&self) -> bool {
    unsafe { NT_GetTopicRetained(self.handle) != 0 }
  }

  pub fn exists(&self) -> bool {
    unsafe { NT_GetTopicExists(self.handle) != 0 }
  }

  pub fn subscribe<V: Value>(&self) -> Subscriber<V> {
    let ts = CString::new(V::NT_TYPE_STRING.to_owned()).unwrap();
    let s = unsafe { NT_Subscribe(self.handle, V::NT_TYPE.into(), ts.as_ptr(), &DEFAULT_PUBSUB) };
    Subscriber { handle: s, value_t: PhantomData }
  }

  pub fn publish<V: Value>(&self) -> Publisher<V> {
    let ts = CString::new(V::NT_TYPE_STRING.to_owned()).unwrap();
    let p = unsafe { NT_Publish(self.handle, V::NT_TYPE.into(), ts.as_ptr(), &DEFAULT_PUBSUB) };
    Publisher { handle: p, value_t: PhantomData }
  }

  pub fn entry<V: Value>(&self) -> Entry<V> {
    let ts = CString::new(V::NT_TYPE_STRING.to_owned()).unwrap();
    let e = unsafe { NT_GetEntryEx(self.handle, V::NT_TYPE.into(), ts.as_ptr(), &DEFAULT_PUBSUB) };
    Entry { handle: e, value_t: PhantomData }
  }

  pub fn subscribe_struct<T: NTStruct>(&self) -> StructSubscriber<T> {
    let ts = CString::new(T::get_full_type_string()).unwrap();
    T::publish_schema(self.instance_handle);
    let s = unsafe { NT_Subscribe(self.handle, robot_rs_ntcore_sys::NT_Type::NT_RAW, ts.as_ptr(), &DEFAULT_PUBSUB) };
    StructSubscriber { handle: s, value_t: PhantomData }
  }

  pub fn publish_struct<T: NTStruct>(&self) -> StructPublisher<T> {
    let ts = CString::new(T::get_full_type_string()).unwrap();
    T::publish_schema(self.instance_handle);
    let p = unsafe { NT_Publish(self.handle, robot_rs_ntcore_sys::NT_Type::NT_RAW, ts.as_ptr(), &DEFAULT_PUBSUB) };
    StructPublisher { handle: p, value_t: PhantomData }
  }
}
