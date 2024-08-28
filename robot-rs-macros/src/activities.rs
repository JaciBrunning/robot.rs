use proc_macro::TokenStream;
use proc_macro2::Span;
use quote::quote;

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
      impl<#(#types: Send + 'static),*> #trait_ident for (#(Arc<System<#types>>,)*) {
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
              tokio::join!(
                #(self.#is.on_activity_finished(vals.#is)),*
              );
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
