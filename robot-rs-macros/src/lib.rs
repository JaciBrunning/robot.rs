mod activities;

#[proc_macro_derive(Systems, attributes(system))]
pub fn derive_systems(input: proc_macro::TokenStream) -> proc_macro::TokenStream {
  activities::derive_systems(input)
}

#[proc_macro]
pub fn impl_perform_for_tuple(item: proc_macro::TokenStream) -> proc_macro::TokenStream {
  activities::impl_perform_for_tuple(item)
}