use std::ffi::CString;

use crate::{nt_internal::{NT_Inst, NT_GetDefaultInstance, NT_CreateInstance, NT_DestroyInstance, NT_StartServer, NT_StopServer}, topic::Topic};

#[derive(Debug, Clone)]
pub struct ServerConfig<'a> {
  pub persist_filename: &'a str,
  pub listen_address: &'a str,
  pub port3: usize,
  pub port4: usize
}

impl<'a> Default for ServerConfig<'a> {
  fn default() -> Self {
    Self {
      persist_filename: "networktables.json",
      listen_address: "",
      port3: 1735,
      port4: 5810
    }
  }
}

pub struct NetworkTableInstance {
  pub(crate) handle: NT_Inst
}

impl NetworkTableInstance {
  pub fn new() -> Self {
    Self { handle: unsafe { NT_CreateInstance() } }
  }

  pub fn start_server<'a>(&mut self, config: ServerConfig<'a>) {
    let cstr_persist = CString::new(config.persist_filename).unwrap();
    let cstr_listen = CString::new(config.listen_address).unwrap();
    
    unsafe {
      NT_StartServer(
        self.handle,
        cstr_persist.as_ptr(),
        cstr_listen.as_ptr(),
        1735,
        5810
      )
    }
  }

  pub fn stop_server(&mut self) {
    unsafe { NT_StopServer(self.handle) }
  }

  pub fn topic(&self, name: &str) -> Topic {
    Topic::new(&self, name)
  }
}

impl Default for NetworkTableInstance {
  fn default() -> Self {
    Self { handle: unsafe { NT_GetDefaultInstance() } }
  }
}

impl Drop for NetworkTableInstance {
  fn drop(&mut self) {
    let handle = self.handle;
    if handle != unsafe { NT_GetDefaultInstance() } {
      unsafe { NT_DestroyInstance(handle) };
    }
  }
}