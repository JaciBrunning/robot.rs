mod activities;

#[proc_macro]
pub fn impl_perform_for_tuple(item: proc_macro::TokenStream) -> proc_macro::TokenStream {
  activities::impl_perform_for_tuple(item)
}
