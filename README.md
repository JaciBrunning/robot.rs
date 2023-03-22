robot.rs
====
An experimental FRC robotics framework, written in Rust.

## Notes on Getting Started
Currently, this is just experimental, so getting it running takes a little bit of legwork. Foremostly, there are a set of libs from WPI that have to be copied into the `wpilib-hal` project. Copy the following libs for your platform to `wpilib-hal/libs/` and its associated headers to `wpilib-hal/include/`:
- ntcore
- wpiHal
- wpimath
- wpinet
- wpiutil

You can also optionally include `halsim_gui` and set the `HALSIM_EXTENSIONS` environment variables if you want to include a simulation GUI.

## A few examples
### Composable IO
I/O in `robot.rs` is composable - meaning you can wrap IO in other structs to change behaviour.
```rust
// Clamp a motor's output to -10V..10V
let motor = ClampedMotor(PWMSparkMax::new(0), -10, 10);

// Create a new digital output, that's inverted.
let out = InvertOutput(DigitalRoboRIO::new(1).output());
```

### Simple Joystick Motor Controller
You can run `robot.rs` in synchronous mode by implementing code the same way you would in any other program. 

```rust
pub fn my_robot(running: Arc<AtomicBool>) -> RobotResult {
  let mut motor = PWMSparkMax::new(0);
  let xbox = Xbox::new(0);

  // While the program is running... (Atomic load)
  while running.load(std::sync::atomic::Ordering::Relaxed) {
    // Get the Left X input from the motor and send it to a motor
    let value = xbox.left_x().get();
    motor.set_speed(value);
    std::thread::sleep(std::time::Duration::from_millis(20));
  }
  Ok(())
}

robot_main!(my_robot);
```

### Async Auto
`robot.rs` is compatible with the asynchronous framework tokio, allowing you to write autonomous programs like this:

```rust
async fn two_piece_auto(elevator: Arc<Elevator>, drivetrain: Arc<Drivetrain>, gripper: Arc<Gripper>) {
  info!("Two Cube Auto Begin");
  // Drop the first gamepiece
  gripper.release().await;
  info!("That's Cube One!");

  // Turn 180
  drivetrain.drive_distance(-0.1).await;
  drivetrain.turn_angle(180.0).await;

  // Run at the same time - drop elevator to 0.2 and drive to the setpoint
  let elev = elevator.go_to_height(0.2);
  let drive = async {
    drivetrain.drive_distance(0.5).await;
    drivetrain.turn_angle(90.0).await;
    drivetrain.drive_distance(0.2).await;
  };
  join!(elev, drive);

  // Grab the gamepiece
  gripper.grip().await;
  info!("Grabbed Cube Two");

  // Raise the elevator and back off
  let elev = elevator.go_to_height(1.0);
  let drive = drivetrain.drive_distance(-0.5);
  join!(elev, drive);

  // Turn back towards the scoring area, drive over
  drivetrain.turn_angle(90.0).await;
  drivetrain.drive_distance(0.5).await;

  // Drop the 2nd gamepiece
  gripper.release().await;
  info!("That's Cube Two!");
  info!("Two Cube Auto Stop");
}
```

### Async Elevator
Simple asynchronous example of an elevator subsystem. Notice the use of a 'Control Lock' - `robot.rs`'s solution to resource control to allow multiple systems access to the same subsystem, but not at the same time. Resources can 'steal' the system's control lock, granting them write access to the system and revoking the write access of the last system. This is akin to WPILib's "Requires" in command-based programming.

```rust
async fn auto_routine<'a, E: AbstractElevator<'a>>(elevator: &'a E) -> ElevatorResult<()> {
  info!("Auto Start");
  let control = elevator.control().steal().await;

  // Note the `.await?`. The question-mark operator allows the auto routine to bail early
  // if the control lock gets stolen, or the elevator otherwise runs into an error.
  elevator.go_to_height(0.5, &control).await?;
  elevator.go_to_height(1.0, &control).await?;
  elevator.go_to_height(0.75, &control).await?;
  elevator.go_to_height(0.0, &control).await?;
  elevator.go_to_height(1.0, &control).await?;

  Ok(())
}


impl Elevator {
  // ...
  pub async fn run(&self) {
    loop {
      let mut demand_voltage = 0.0;
      let current_height = self.config.height_sensor.read().await.get_distance();
      let mut pid = self.pid.write().await;

      match &*self.state.read().await {
        ElevatorState::Idle => pid.reset(),
        ElevatorState::Manual { voltage } => {
          pid.reset();
          demand_voltage = *voltage
        },
        ElevatorState::HeightControl { height } => {
          pid.set_setpoint(*height);
          
          let feedforward = self.config.motor_model.voltage(
            self.config.mass * 9.81 * self.config.spool_radius,
            0.0
          );

          demand_voltage = pid.calculate(current_height, now()).output + feedforward;
        }
      }

      // Publish to NetworkTables, with updated PID coefficients as required
      nt!("elevator/state", format!("{:?}", self.state.read().await)).unwrap();
      nt!("elevator/height", current_height).unwrap();
      nt!("elevator/voltage", demand_voltage).unwrap();
      pid.nt_update("elevator");

      self.config.motor.write().await.set_voltage(demand_voltage);
      tokio::time::sleep(tokio::time::Duration::from_millis(20)).await;
    }
  }
  // ...
}
```