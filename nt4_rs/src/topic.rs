use std::{ffi::CString};

use wpilib_hal::{NT_Topic, NT_GetTopic, NT_GetTopicType, NT_GetTopicTypeString, NT_SetTopicPersistent, NT_GetTopicPersistent, NT_SetTopicRetained, NT_GetTopicRetained, NT_GetTopicExists};

use crate::{instance::NetworkTableInstance, types::Type};

struct Topic {
  handle: NT_Topic
}

impl Topic {
  pub fn new(instance: &NetworkTableInstance, name: &str) -> Self {
    let cstr = CString::new(name).unwrap();
    Self {
      handle: unsafe {
        NT_GetTopic(instance.handle, cstr.as_ptr(), cstr.as_bytes().len() as u64)
      }
    }
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

  // TODO: Properties, Pub, Sub
}