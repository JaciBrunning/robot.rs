use std::time::Duration;

use mockall::{automock, Sequence};
use robot_rs::{activity::{Systems, Priority}, perform, activity_factory};
use tokio::sync::mpsc;
use futures::join;

pub struct DropNotifier<F: FnOnce()> {
  on_finish: Option<F>
}

impl<F: FnOnce()> Drop for DropNotifier<F> {
  fn drop(&mut self) {
    (self.on_finish.take().unwrap())()
  }
}

#[automock]
pub trait ActivityNotifier {
  fn start(&self) -> ();
  fn stop(&self) -> ();
  fn drop(&self) -> ();
}

struct Drivetrain<A: ActivityNotifier>( (A, mpsc::Receiver<()>), (A, mpsc::Receiver<()>) );
struct Elevator<A: ActivityNotifier>( (A, mpsc::Receiver<()>), (A, mpsc::Receiver<()>) );

impl<A: ActivityNotifier> Drivetrain<A> {
  pub async fn task_1(&mut self) {
    let d = DropNotifier { on_finish: Some(|| { self.0.0.drop() }) };
    self.0.0.start();
    self.0.1.recv().await.unwrap();
    self.0.0.stop();
    drop(d)
  }
  pub async fn task_2(&mut self) {
    let d = DropNotifier { on_finish: Some(|| { self.1.0.drop() }) };
    self.1.0.start();
    self.1.1.recv().await.unwrap();
    self.1.0.stop();
    drop(d)
  }
}

impl<A: ActivityNotifier> Elevator<A> {
  pub async fn task_1(&mut self) {
    let d = DropNotifier { on_finish: Some(|| { self.0.0.drop() }) };
    self.0.0.start();
    self.0.1.recv().await.unwrap();
    self.0.0.stop();
    drop(d)
  }
  pub async fn task_2(&mut self) {
    let d = DropNotifier { on_finish: Some(|| { self.1.0.drop() }) };
    self.1.0.start();
    self.1.1.recv().await.unwrap();
    self.1.0.stop();
    drop(d)
  }
}

async fn uses_both<A: ActivityNotifier>((dt, el): (&mut Drivetrain<A>, &mut Elevator<A>)) {
  let fut1 = dt.task_2();
  let fut2 = el.task_2();

  join!(fut1, fut2);

  ()
}

#[derive(Systems)]
struct MySystems<A: ActivityNotifier, B: ActivityNotifier> {
  drivetrain: Drivetrain<A>,
  elevator: Elevator<B>
}

#[derive(Systems)]
struct SingleSystem<A: ActivityNotifier> {
  drivetrain: Drivetrain<A>,
}

#[tokio::test]
async fn test_always_pass() {
  let mut an1 = MockActivityNotifier::new();
  let mut an2 = MockActivityNotifier::new();

  let mut seq = mockall::Sequence::new();

  an1.expect_start()
    .once()
    .in_sequence(&mut seq)
    .return_const(());

  an1.expect_stop()
    .once()
    .in_sequence(&mut seq)
    .return_const(());

  an1.expect_drop()
    .once()
    .in_sequence(&mut seq)
    .return_const(());

  an2.expect_start().never();
  an2.expect_stop().never();
  an2.expect_drop().never();

  let (tx1, rx1) = mpsc::channel(1);
  let (_tx2, rx2) = mpsc::channel(1);
  let mut dt = Drivetrain((an1, rx1), (an2, rx2));
  tokio::task::spawn(async move { dt.task_1().await });

  tokio::time::sleep(Duration::from_millis(10)).await;
  tx1.send(()).await.unwrap();
  tokio::time::sleep(Duration::from_millis(10)).await;
}

#[tokio::test]
async fn independent() {
  let dt_channels = (mpsc::channel(1), mpsc::channel(1));
  let el_channels = (mpsc::channel(1), mpsc::channel(1));
  
  let mut dt = Drivetrain(( MockActivityNotifier::new(), dt_channels.0.1 ), (MockActivityNotifier::new(), dt_channels.1.1));
  let mut el = Elevator(( MockActivityNotifier::new(), el_channels.0.1 ), (MockActivityNotifier::new(), el_channels.1.1));

  // Setup mock
  let mut seq = Sequence::new();

  dt.0.0.expect_start().once().in_sequence(&mut seq).return_const(());
  el.0.0.expect_start().once().in_sequence(&mut seq).return_const(());

  dt.0.0.expect_stop().once().in_sequence(&mut seq).return_const(());
  dt.0.0.expect_drop().once().in_sequence(&mut seq).return_const(());

  dt.1.0.expect_start().once().in_sequence(&mut seq).return_const(());
  
  el.0.0.expect_stop().never();
  el.0.0.expect_drop().once().in_sequence(&mut seq).return_const(());
  
  el.1.0.expect_start().once().in_sequence(&mut seq).return_const(());

  dt.1.0.expect_stop().once().in_sequence(&mut seq).return_const(());
  dt.1.0.expect_drop().once().in_sequence(&mut seq).return_const(());

  el.1.0.expect_stop().once().in_sequence(&mut seq).return_const(());
  el.1.0.expect_drop().once().in_sequence(&mut seq).return_const(());

  // let dt_an = ()
  let systems = MySystems { drivetrain: dt, elevator: el }.shared();

  perform!(systems.drivetrain, Priority(1), Drivetrain::task_1);
  tokio::time::sleep(Duration::from_millis(10)).await;
  perform!(systems.elevator, Priority(1), Elevator::task_1);
  tokio::time::sleep(Duration::from_millis(10)).await;

  // Stop the drivetrain, but not the elevator
  dt_channels.0.0.send(()).await.unwrap();
  tokio::time::sleep(Duration::from_millis(10)).await;

  perform!(systems.drivetrain, Priority(2), Drivetrain::task_2);
  tokio::time::sleep(Duration::from_millis(10)).await;
  perform!(systems.elevator, Priority(2), Elevator::task_2);
  tokio::time::sleep(Duration::from_millis(10)).await;

  dt_channels.1.0.send(()).await.unwrap();
  tokio::time::sleep(Duration::from_millis(10)).await;
  el_channels.1.0.send(()).await.unwrap();
  tokio::time::sleep(Duration::from_millis(10)).await;
}

#[tokio::test]
async fn double_system() {
  let dt_channels = (mpsc::channel(1), mpsc::channel(1));
  let el_channels = (mpsc::channel(1), mpsc::channel(1));
  
  let mut dt = Drivetrain(( MockActivityNotifier::new(), dt_channels.0.1 ), (MockActivityNotifier::new(), dt_channels.1.1));
  let mut el = Elevator(( MockActivityNotifier::new(), el_channels.0.1 ), (MockActivityNotifier::new(), el_channels.1.1));

  // Setup mock
  let mut seq = Sequence::new();

  dt.0.0.expect_start().once().in_sequence(&mut seq).return_const(());
  el.0.0.expect_start().once().in_sequence(&mut seq).return_const(());

  dt.0.0.expect_stop().once().in_sequence(&mut seq).return_const(());
  dt.0.0.expect_drop().once().in_sequence(&mut seq).return_const(());

  el.0.0.expect_drop().once().in_sequence(&mut seq).return_const(());

  dt.1.0.expect_start().once().in_sequence(&mut seq).return_const(());

  el.0.0.expect_stop().never();

  el.1.0.expect_start().once().in_sequence(&mut seq).return_const(());

  dt.1.0.expect_stop().once().in_sequence(&mut seq).return_const(());
  dt.1.0.expect_drop().once().in_sequence(&mut seq).return_const(());

  el.1.0.expect_stop().once().in_sequence(&mut seq).return_const(());
  el.1.0.expect_drop().once().in_sequence(&mut seq).return_const(());

  let systems = MySystems { drivetrain: dt, elevator: el }.shared();

  perform!(systems.drivetrain, Priority(1), Drivetrain::task_1);
  tokio::time::sleep(Duration::from_millis(10)).await;
  perform!(systems.elevator, Priority(1), Elevator::task_1);
  tokio::time::sleep(Duration::from_millis(10)).await;

  // Stop the drivetrain, but not the elevator
  dt_channels.0.0.send(()).await.unwrap();
  tokio::time::sleep(Duration::from_millis(10)).await;

  perform!((systems.drivetrain, systems.elevator), Priority(2), uses_both);
  tokio::time::sleep(Duration::from_millis(10)).await;
  
  dt_channels.1.0.send(()).await.unwrap();
  tokio::time::sleep(Duration::from_millis(10)).await;
  el_channels.1.0.send(()).await.unwrap();
  tokio::time::sleep(Duration::from_millis(10)).await;
}

#[tokio::test]
async fn idle_task() {
  let dt_channels = (mpsc::channel(1), mpsc::channel(1));
  
  let mut dt = Drivetrain(( MockActivityNotifier::new(), dt_channels.0.1 ), (MockActivityNotifier::new(), dt_channels.1.1));

  let mut seq = Sequence::new();
  dt.0.0.expect_start().once().in_sequence(&mut seq).return_const(());
  dt.0.0.expect_stop().once().in_sequence(&mut seq).return_const(());
  dt.0.0.expect_drop().once().in_sequence(&mut seq).return_const(());

  dt.1.0.expect_start().once().in_sequence(&mut seq).return_const(());
  dt.1.0.expect_drop().once().in_sequence(&mut seq).return_const(());

  dt.0.0.expect_start().once().in_sequence(&mut seq).return_const(());
  dt.0.0.expect_drop().once().in_sequence(&mut seq).return_const(());

  let systems = SingleSystem { drivetrain: dt }.shared();

  systems.drivetrain.set_idle_activity(activity_factory!(Drivetrain::task_2));
  perform!(systems.drivetrain, Priority(1), Drivetrain::task_1);
  tokio::time::sleep(Duration::from_millis(10)).await;

  dt_channels.0.0.send(()).await.unwrap();
  tokio::time::sleep(Duration::from_millis(10)).await;

  perform!(systems.drivetrain, Priority(1), Drivetrain::task_1);
  tokio::time::sleep(Duration::from_millis(10)).await;
}