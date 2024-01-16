use std::{error::Error, sync::{atomic::AtomicBool, Arc}};

use log::{error, warn, info};
use ntcore_rs::{NetworkTableInstance, ServerConfig};
use crate::ds::{observe, RobotControlState};
#[cfg(feature = "hal")]
use crate::hal::{HAL_Initialize, HAL_SetNotifierThreadPriority, hal_safe_call, HAL_HasMain, HAL_Shutdown, HAL_ExitMain, HAL_RunMain};

#[derive(Clone)]
pub struct RobotState {
  pub(crate) inner: Arc<AtomicBool>
}

impl RobotState {
  pub fn running(&self) -> bool {
    self.inner.load(std::sync::atomic::Ordering::Relaxed)
  }

  #[cfg(feature = "hal")]
  pub fn get(&self) -> RobotControlState {
    RobotControlState::current()
  }
}

#[macro_export]
macro_rules! robot_main {
  ($func:ident) => {
    use robot_rs::start::init_all;

    pub fn main() {
      init_all($func);
    }
  };
  (async $func:ident) => {
    use robot_rs::start::init_all;
    use std::sync::atomic::AtomicBool;

    pub fn main() {
      init_all(async_main);
    }

    #[tokio::main]
    pub async fn async_main(running: RobotState) -> RobotResult {
      let fut = $func(running.clone());

      tokio::select! {
        result = fut => result,
        _ = async {
          loop {
            if !running.running() {
              return;
            }
            tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
          }
        } => Ok(())
      }
    }
  }
}

pub type RobotResult = Result<(), Box<dyn Error>>;

pub fn init_all<F: FnOnce(RobotState) -> Result<(), Box<dyn Error>> + Send + 'static>(f: F) {
  log_init();

  info!("Initializing HAL...");

  hal_init();

  info!("**** Running Robot ****");

  NetworkTableInstance::default().start_server(ServerConfig::default());

  observe::start();

  let running = Arc::new(AtomicBool::new(true));

  #[cfg(feature = "hal")]
  {
    let hal_has_main = unsafe { HAL_HasMain() } != 0;
    if hal_has_main {
      // Spawn a new thread for the user program. This is commonly used in
      // simulation with sim extensions

      info!("HAL has main. Running user program in a new thread.");

      let r2 = RobotState { inner: running.clone() };
      let user_thread = std::thread::spawn(move || {
        match f(r2) {
          Ok(()) => { warn!("Robot Exited Gracefully") },
          Err(e) => error!("Robot Error: {}", e)
        };

        unsafe { HAL_ExitMain() };
      });

      unsafe { HAL_RunMain(); }

      running.store(false, std::sync::atomic::Ordering::Relaxed);

      user_thread.join().unwrap();
    } else {
      // Run the user program where it is
      match f(RobotState { inner: running }) {
        Ok(()) => {
          warn!("Robot Exited Gracefully")
        },
        Err(e) => {
          error!("Robot Error: {}", e)
        }
      }
    }
  }

  #[cfg(not(feature = "hal"))]
  {
    match f(RobotState { inner: running }) {
      Ok(()) => {
        warn!("Robot Exited Gracefully")
      },
      Err(e) => {
        error!("Robot Error: {}", e)
      }
    }
  }

  #[cfg(feature = "hal")]
  unsafe { HAL_Shutdown() };
}

pub fn log_init() {
  env_logger::builder().filter_level(log::LevelFilter::Info).target(env_logger::Target::Stdout).init();
}

#[cfg(feature = "hal")]
pub fn hal_init() {
  unsafe {
    if HAL_Initialize(500, 0) == 0 {
      panic!("Could not initialise HAL");
    }

    match hal_safe_call!(HAL_SetNotifierThreadPriority(1, 40)) {
      Ok(_) => (),
      Err(e) => error!("Could not set notifier thread to priority 40: {}", e),
    }
  }
}

#[cfg(not(feature = "hal"))]
pub fn hal_init() { }