use nt4_rs::{instance::NetworkTableInstance, nt};

fn main() {
    let mut instance = NetworkTableInstance::default();
    instance.start_server(Default::default());

    nt!("/my/entry", 1234).unwrap();
    loop {
        println!("{:?}", nt!(read "/my/entry"));
        std::thread::sleep(std::time::Duration::from_millis(1000));
    }
}
