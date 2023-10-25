use std::{ffi::CString, fmt::Debug};

use wpilib_hal::{
    NT_Entry, NT_GetEntry, NT_GetEntryName, NT_GetEntryType, NT_GetEntryValue, NT_SetEntryValue,
    NT_Value,
};

use crate::{
    instance::NetworkTableInstance,
    types::{NTResult, Type, Value, ValueContext},
};

pub struct Entry {
    handle: NT_Entry,
}

impl Entry {
    pub fn new(inst: &NetworkTableInstance, name: &str) -> Self {
        let cstr = CString::new(name).unwrap();
        Self {
            handle: unsafe {
                NT_GetEntry(inst.handle, cstr.as_ptr(), cstr.as_bytes().len() as u64)
            },
        }
    }

    pub fn get_type(&self) -> Type {
        unsafe { NT_GetEntryType(self.handle) }.into()
    }

    pub fn exists(&self) -> bool {
        self.get_type() != Type::Unassigned
    }

    pub fn get_name(&self) -> String {
        let mut len = 0;
        let buf = unsafe { NT_GetEntryName(self.handle, &mut len) };
        std::str::from_utf8(unsafe { std::slice::from_raw_parts(buf as *const u8, len as usize) })
            .unwrap()
            .to_owned()
    }

    // TODO: Get / Set Persistent, Unpublish, Topic

    pub fn get(&self) -> ValueContext {
        let mut value = NT_Value::default();
        unsafe { NT_GetEntryValue(self.handle, &mut value) };
        ValueContext::from(value)
    }

    pub fn set<I: Into<Value>>(&mut self, value: I) -> NTResult<()> {
        let v: Value = value.into();
        v.with_nt_value(|ntv| {
            let err = unsafe { NT_SetEntryValue(self.handle, &ntv as *const NT_Value) };
            if err == 0 {
                Err(crate::types::NTError::TypeMismatch)
            } else {
                Ok(())
            }
        })
    }
}

impl Debug for Entry {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Entry")
            .field("name", &self.get_name())
            .field("exists", &self.exists())
            .field("value", &self.get())
            .finish()
    }
}
