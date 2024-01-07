pub trait Wrapper<Inner> {
  fn eject(self) -> Inner;
}
