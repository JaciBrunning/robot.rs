use std::{marker::PhantomData, sync::{atomic::AtomicUsize, Arc}};

use tokio::sync::{RwLock, RwLockWriteGuard};

// Resources will require a control lock to operate
// A control lock can be stolen (akin to interrupted), or requested.
// Only if the control lock is valid will access to the resource be granted.
// It's essentially a mutex that can be stolen from another piece of code

pub struct ControlLockResource<'a, T> {
  next_index: AtomicUsize,
  owner: Arc<RwLock<Option<usize>>>,
  _phantom: PhantomData<&'a T>
}

// Taken from the resource, 
pub struct ControlLock<'a, T> {
  me: usize,
  owner: Arc<RwLock<Option<usize>>>,
  _phantom: PhantomData<&'a T>
}

pub struct ControlLockExclusive<'a, T> {
  #[allow(dead_code)]
  guard: RwLockWriteGuard<'a, Option<usize>>,
  _phantom: PhantomData<T>
}

impl<'a, T> ControlLockResource<'a, T> {
  pub fn new() -> Self {
    Self {
      next_index: AtomicUsize::new(0),
      owner: Arc::new(RwLock::new(None)),
      _phantom: Default::default()
    }
  }

  // Forcibly take
  pub async fn steal(&self) -> ControlLock<T> {
    let mut owner = self.owner.write().await;
    let idx = self.next_index.load(std::sync::atomic::Ordering::Relaxed);
    // *self.owner.write().await = Some(idx);
    *owner = Some(idx);
    self.next_index.store(idx + 1, std::sync::atomic::Ordering::Relaxed);
    
    ControlLock {
      me: idx,
      owner: self.owner.clone(),
      _phantom: Default::default()
    }
  }

  // Only if there is no existing lock
  pub async fn request(&self) -> Option<ControlLock<T>> {
    let mut owner = self.owner.write().await;
    if owner.is_none() {
      let idx = self.next_index.load(std::sync::atomic::Ordering::Relaxed);
      *owner = Some(idx);
      self.next_index.store(idx + 1, std::sync::atomic::Ordering::Relaxed);
      
      Some(ControlLock {
        me: idx,
        owner: self.owner.clone(),
        _phantom: Default::default()
      })
    } else {
      None
    }
  }
}

impl<'a, T> Default for ControlLockResource<'a, T> {
  fn default() -> Self {
    Self::new()
  }
}

impl<'a, T> ControlLock<'a, T> {
  pub async fn is_mine(&'a self) -> bool {
    *self.owner.read().await == Some(self.me)
  }

  pub async fn get(&'a self) -> Option<ControlLockExclusive<'a, T>> {
    let mut owner = self.owner.write().await;

    if *owner == Some(self.me) {
      // It's ours - let's use it
      Some(ControlLockExclusive { guard: owner, _phantom: Default::default() })
    } else if owner.is_none() {
      // No one's got it, so we might as well have it
      // *self.owner.write().await = Some(self.me);
      *owner = Some(self.me);
      Some(ControlLockExclusive { guard: owner, _phantom: Default::default() })
    } else {
      None
    }
  }
}

impl<'a, T> Drop for ControlLock<'a, T> {
  fn drop(&mut self) {
    let owner_arc = self.owner.clone();
    let me = self.me;

    tokio::task::spawn(async move {
      let owner = owner_arc.read().await;
      if *owner == Some(me) {
        // If we own it, upgrade to a write guard
        drop(owner);

        let mut owner = owner_arc.write().await;
        // In the time we've upgraded, someone might have stolen the lock
        if *owner == Some(me) {
          *owner = None;
        }
      }
    });
  }
}
