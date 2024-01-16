use std::{sync::{Arc, RwLock}, pin::Pin, time::Duration};

use futures::Future;
use tokio::sync::{oneshot, Mutex};

#[macro_export]
macro_rules! activity_factory {
  ($fnpath:expr) => {
    |x| futures::future::FutureExt::boxed(async move { $fnpath(x).await })
  }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Priority(pub usize);

#[macro_export]
macro_rules! system {
  ($system:expr) => {
    std::sync::Arc::new(robot_rs::activity::System::new($system))
  };
}

pub type Shared<T> = Arc<RwLock<T>>;

pub fn make_shared<T>(t: T) -> Shared<T> {
  Arc::new(RwLock::new(t))
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
  storage: Mutex<MaybeReferred<T>>,
  // Use a standard rwlock since we don't access it often, and std is faster than async.
  default_activity_factory: Arc<std::sync::RwLock<Option<Arc<dyn Fn(&mut T) -> Pin<Box<dyn Future<Output = ()> + '_ + Send>> + Send + Sync>>>>
}

impl<T> System<T> {
  pub fn new(system: T) -> Self {
    Self { storage: Mutex::new(MaybeReferred::Owned(system)), default_activity_factory: Arc::new(std::sync::RwLock::new(None)) }
  }

  pub fn set_idle_activity<F: Fn(&mut T) -> Pin<Box<dyn Future<Output = ()> + '_ + Send>> + Send + Sync + 'static>(&self, f: F) {
    *self.default_activity_factory.write().unwrap() = Some(Arc::new(f));
  }

  pub async fn on_activity_finished(self: Arc<Self>, mut sys: T) {
    let factory = {
      self.default_activity_factory.read().unwrap().clone()
    };
    match factory {
      Some(factory) => {
        let (tx, rx) = oneshot::channel();
        *self.storage.lock().await = MaybeReferred::Referred(Priority(0), tx);

        let future = async {
          loop {
            factory(&mut sys).await;
            tokio::time::sleep(Duration::from_millis(1)).await;  // Just in case the above exits immediately
          }
        };
        
        tokio::select! {
          _ = future => panic!("A future that can never finish has finished!"),
          new_sender = rx => {
            new_sender.unwrap().send(sys).ok();
          }
        }
      },
      None => {
        *self.storage.lock().await = MaybeReferred::Owned(sys);
      }
    }
  }

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
          self.on_activity_finished(sys).await;
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
  ($system:expr, $priority:expr, $func:path) => {
    perform!($system, $priority, |x| futures::future::FutureExt::boxed(async move { $func(x).await }))
  };
  ($system:expr, $priority:expr, $func:expr) => {
    {
      use robot_rs::activity::tup_ext::*;
      tokio::task::spawn($system.clone().perform($priority, $func))
    }
  };
}

