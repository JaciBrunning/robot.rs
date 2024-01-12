use std::{sync::Arc, pin::Pin};

use futures::Future;
pub use robot_rs_macros::Systems;
use tokio::sync::{oneshot, Mutex};

#[macro_export]
macro_rules! pinbox {
  ($fnpath:expr) => {
    |x| futures::future::FutureExt::boxed(async move { $fnpath(x).await })
  }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Priority(pub usize);

pub trait Systems {
  type Shared;

  fn shared(self) -> Self::Shared;
}

pub enum MaybeReferred<T> {
  Owned(T),
  Referred(Priority, oneshot::Sender<oneshot::Sender<T>>)
}

impl<T> MaybeReferred<T> {
  pub fn can_take(&mut self, priority: Priority) -> bool {
    match self {
      MaybeReferred::Owned(_) => true,
      MaybeReferred::Referred(p, _) if *p < priority => true,
      _ => false
    }
  }

  pub async fn try_take(&mut self, priority: Priority, take_back: oneshot::Sender<oneshot::Sender<T>>) -> Option<T> {
    if self.can_take(priority) {
      let old = std::mem::replace(self, MaybeReferred::Referred(priority, take_back));
      match old {
        MaybeReferred::Owned(o) => Some(o),
        MaybeReferred::Referred(_, taker) => {
          // Take it from another task
          let (tx, rx) = oneshot::channel();
          taker.send(tx).ok();
          Some(rx.await.unwrap())
        },
      }
    } else {
      None
    }
  }
}

pub struct System<T> {
  storage: Mutex<MaybeReferred<T>>
}

impl<T> System<T> {
  pub fn new(system: T) -> Self {
    Self { storage: Mutex::new(MaybeReferred::Owned(system)) }
  }

  // pub async fn perform<O, Fut: Future<Output = O>, F: FnOnce(&mut T) -> Fut>(&self, priority: Priority, f: F) -> Option<O> {
  pub async fn perform<O, F>(self: Arc<Self>, priority: Priority, f: F) -> Option<O>
    where F: FnOnce(&mut T) -> Pin<Box<dyn Future<Output = O> + '_ + Send>>
  {
    let (tx_take, rx_take) = oneshot::channel();

    let val = {
      self.storage.lock().await.try_take(priority, tx_take).await
    };

    if let Some(mut sys) = val {
      let future = f(&mut sys);

      tokio::select! {
        ret = future => {
          // Reset the storage to be owned
          *self.storage.lock().await = MaybeReferred::Owned(sys);
          Some(ret)
        },
        new_sender = rx_take => {
          new_sender.unwrap().send(sys).ok();
          None
        }
      }
    } else {
      None
    }
  }
}

pub mod tup_ext {
  use robot_rs_macros::impl_perform_for_tuple;
  use std::pin::Pin;
  use futures::Future;
  use super::*;
  
  impl_perform_for_tuple!();
}

#[macro_export]
macro_rules! perform {
  ($system:expr, $priority:expr, $func:expr) => {
    {
      use robot_rs::activity::tup_ext::*;
      tokio::task::spawn($system.clone().perform($priority, $func))
    }
  }
}

