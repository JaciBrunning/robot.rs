macro_rules! wrapped_traits {  
  ($trait:ident, $class:ident) => {
    impl<D: $trait> $class<D> {
      pub fn revert(self) -> D {
        self.0
      }
    }

    impl<D: $trait> std::ops::Deref for $class<D> {
      type Target = D;

      fn deref(&self) -> &Self::Target {
        &self.0
      }
    }

    impl<D: $trait> std::ops::DerefMut for $class<D> {
      fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
      }
    }
  }
}
pub(crate) use wrapped_traits;

macro_rules! wrapped_traits_nogen {
  ($class:ident, $target:ident) => {
    impl $class {
      pub fn revert(self) -> $target {
        self.0
      }
    }
    
    impl std::ops::Deref for $class {
      type Target = $target;

      fn deref(&self) -> &Self::Target {
        &self.0
      }
    }

    impl std::ops::DerefMut for $class {
      fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
      }
    }
  }
}
pub(crate) use wrapped_traits_nogen;
