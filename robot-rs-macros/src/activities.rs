use darling::FromField;
use proc_macro::TokenStream;
use proc_macro2::Span;
use syn::{DeriveInput, parse_macro_input, Ident, Type, Visibility};
use quote::quote;

#[derive(Debug, FromField)]
#[darling(attributes(marshal))]
struct StructFieldReceiver {
  ident: Option<Ident>,
  ty: Type,
  vis: Visibility,
}

pub fn derive_systems(input: proc_macro::TokenStream) -> proc_macro::TokenStream {
  let DeriveInput {
    attrs: _, vis, ident, generics, data
  } = parse_macro_input!(input as DeriveInput);

  let (impl_gen, ty_gen, where_clause) = generics.split_for_impl();

  match data {
    syn::Data::Struct(st) => {
      match st.fields {
        syn::Fields::Named(fields) => {
          let shared_ident = syn::Ident::new(&format!("{}Shared", ident), ident.span());

          let mapped_fields = fields.named.iter().map(|field| StructFieldReceiver::from_field(field).unwrap());

          let shared_fields = mapped_fields.clone().map(|field| {
            let StructFieldReceiver {
              ident: f_ident, ty: f_ty, vis: f_vis
            } = field;

            quote! {
              #f_vis #f_ident: std::sync::Arc<robot_rs::activity::System<#f_ty>>
            }
          });

          let field_initialisers = mapped_fields.clone().map(|field| {
            let StructFieldReceiver {
              ident: f_ident, ty: _, vis: _
            } = field;

            quote! {
              #f_ident: std::sync::Arc::new(robot_rs::activity::System::new(self.#f_ident))
            }
          });

          quote! {
            #vis struct #shared_ident #generics {
              #(#shared_fields),*
            }

            impl #impl_gen robot_rs::activity::Systems for #ident #ty_gen #where_clause {
              type Shared = #shared_ident #ty_gen;
              fn shared(self) -> Self::Shared {
                #shared_ident {
                  #(#field_initialisers),*
                }
              }
            }
          }.into()
        },
        _ => panic!("Non-Named Structs can't be used for commands.")
      }
    },
    _ => panic!("Enums can't be used for commands.")
  }
}

pub fn impl_perform_for_tuple(_item: TokenStream) -> TokenStream {
  let inner = (1..=12).map(|n| {
    let range: Vec<usize> = (0..n).collect();
    let trait_ident = syn::Ident::new(&format!("TuplePerform{}", n), Span::call_site());
    let types: Vec<syn::Ident> = range.iter().map(|i| syn::Ident::new(&format!("T{}", i), Span::call_site())).collect();
    let is: Vec<syn::Index> = range.iter().map(|i| syn::Index::from(*i)).collect();
    let channels = range.iter().map(|_| quote!{ tokio::sync::oneshot::channel() });

    let select_arms = range.iter().map(|i| {
      let others: Vec<syn::Index> = range.iter().filter(|&j| j != i).map(|&j| syn::Index::from(j)).collect();
      let i = syn::Index::from(*i);

      quote!{
        new_sender = channels.#i.1 => {
          #(*self.#others.storage.lock().await = MaybeReferred::Owned(vals.#others);)*

          new_sender.unwrap().send(vals.#i).ok();
          None
        }
      }
    });

    quote! {
      #[async_trait::async_trait]
      pub trait #trait_ident {
        #(type #types;)*

        async fn perform<O: Send, F>(self, priority: Priority, f: F) -> Option<O>
          where F: for<'a> FnOnce( (#(&'a mut Self::#types),*) ) -> Pin<Box<dyn Future<Output = O> + 'a + Send>> + Send;
      }

      #[async_trait::async_trait]
      impl<#(#types: Send),*> #trait_ident for (#(Arc<System<#types>>,)*) {
        #(type #types = #types;)*
        
        async fn perform<O: Send, F>(self, priority: Priority, f: F) -> Option<O>
          where F: for<'a> FnOnce( (#(&'a mut Self::#types),*) ) -> Pin<Box<dyn Future<Output = O> + 'a + Send>> + Send
        {
          let channels = ( #(#channels,)* );
          
          let mut vals = {
            let mut locks = (#(self.#is.storage.lock().await,)*);

            if #(locks.#is.can_take(priority))&&* {
              ( #(locks.#is.try_take(priority, channels.#is.0).await.unwrap(),)* )
            } else {
              return None;
            }
          };

          let future = f(( #(&mut vals.#is),* ));
          tokio::select! {
            ret = future => {
              // Reset the storage to be owned
              #(*self.#is.storage.lock().await = MaybeReferred::Owned(vals.#is);)*
              Some(ret)
            },
            #(#select_arms),*
          }
        }
      }
    } 
  });
  
  let q = quote! {
    #(#inner)*
  };
  q.into()
}