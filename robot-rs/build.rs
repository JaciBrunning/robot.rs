use robot_rs_build_utils::define_environment;

fn main() {
  let target = std::env::var("TARGET").unwrap();

  define_environment(&target);
}
