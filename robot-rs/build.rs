use robot_rs_build_utils::{identify_libraries, copy_deps, define_environment};

fn main() {
  let target = std::env::var("TARGET").unwrap();
  let libraries = identify_libraries(&target, robot_rs_build_utils::LibraryType::Runtime).unwrap();
  copy_deps(libraries).unwrap();

  define_environment(&target);
}