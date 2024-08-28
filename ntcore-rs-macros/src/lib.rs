use darling::FromAttributes;
use quote::quote;
use syn::{parse_macro_input, DeriveInput};

#[derive(Debug, FromAttributes)]
#[darling(attributes(nt))]
struct StructReceiver {
  type_string_fragment: String,
}

#[proc_macro_derive(NTStruct, attributes(nt))]
pub fn derive_marshal(input: proc_macro::TokenStream) -> proc_macro::TokenStream {
  let DeriveInput {
    attrs,
    vis: _,
    ident,
    generics: _,
    data,
  } = parse_macro_input!(input as DeriveInput);

  let our_attrs = StructReceiver::from_attributes(&attrs).unwrap();
  let type_string_fragment = our_attrs.type_string_fragment;

  match data {
    syn::Data::Struct(st) => {
      match st.fields {
        syn::Fields::Named(named) => {
          let mut schema = vec![];
          let mut reads = vec![];
          let mut writes = vec![];
          let mut schema_pubs = vec![];

          for field in named.named.into_iter() {
            let fty = field.ty;
            let fident = field.ident.unwrap();
            let fname = fident.to_string();
            schema.push(quote!{ schema.push(format!("{} {}", <#fty as NTStruct>::TYPE_STRING_FRAG, #fname)) });
            reads.push(quote! { #fident: <#fty as NTStruct>::read(buf)? });
            writes.push(quote! { NTStruct::write(&self.#fident, buf)? });
            schema_pubs.push(quote! { <#fty as NTStruct>::publish_schema(inst) });
          }

          quote! {
            impl NTStruct for #ident {
              const TYPE_STRING_FRAG: &'static str = #type_string_fragment;

              fn get_schema() -> String {
                let mut schema: Vec<String> = vec![];
                #(#schema;)*
                schema.join(";")
              }

              fn publish_schema(inst: u32) {
                let ts = std::ffi::CString::new(Self::get_full_type_string()).unwrap();
                let schema = Self::get_schema();
                let struct_str = std::ffi::CString::new("structschema".to_owned()).unwrap();
                unsafe { robot_rs_ntcore_sys::NT_AddSchema(inst, ts.as_ptr(), struct_str.as_ptr(), schema.as_ptr(), schema.len()) };

                #(#schema_pubs;)*
              }

              fn write(&self, buf: &mut BytesMut) -> std::result::Result<(), Box<dyn std::error::Error>> {
                #(#writes;)*
                Ok(())
              }

              fn read(buf: &mut BytesMut) -> std::result::Result<Self, Box<dyn std::error::Error>> {
                Ok(Self {
                  #(#reads),*
                })
              }
            }
          }.into()
        }
        _ => panic!("NTStruct only works on structs with named fields"),
      }
    }
    _ => panic!("NTStruct only works on structs!"),
  }
}
