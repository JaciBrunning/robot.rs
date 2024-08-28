use std::{
  fmt,
  sync::atomic::{AtomicUsize, Ordering},
};

use env_logger::fmt::Color;

static MAX_MODULE_WIDTH: AtomicUsize = AtomicUsize::new(0);

fn max_target_width(target: &str) -> usize {
  let max_width = MAX_MODULE_WIDTH.load(Ordering::Relaxed);
  if max_width < target.len() {
    MAX_MODULE_WIDTH.store(target.len(), Ordering::Relaxed);
    target.len()
  } else {
    max_width
  }
}

struct Padded<T> {
  value: T,
  width: usize,
}

impl<T: fmt::Display> fmt::Display for Padded<T> {
  fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
    write!(f, "{: <width$}", self.value, width = self.width)
  }
}

pub(crate) fn init() {
  env_logger::builder()
    .filter_level(log::LevelFilter::Info)
    .target(env_logger::Target::Stdout)
    .format(|f, record| {
      use std::io::Write;
      let target = record.target();
      let max_width = max_target_width(target);
      let level = f.default_styled_level(record.level());
      let mut message_style = f.style();

      match record.level() {
        log::Level::Error => {
          message_style.set_bold(true).set_color(Color::Red);
        }
        log::Level::Warn => {
          message_style.set_color(Color::Yellow);
        }
        _ => (),
      };

      let mut style = f.style();
      let target = style.set_bold(true).value(Padded {
        value: target,
        width: max_width,
      });

      let time = f.timestamp_millis();

      writeln!(
        f,
        " {} {:>5} {} > {}",
        time,
        level,
        target,
        message_style.value(format!("{}", record.args()))
      )
    })
    .init();
}
