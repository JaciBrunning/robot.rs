use robot_rs_build_utils::{identify_libraries, copy_deps};

fn main() {
  let libraries = identify_libraries(&std::env::var("TARGET").unwrap(), robot_rs_build_utils::LibraryType::Runtime).unwrap();
  copy_deps(libraries).unwrap()
}