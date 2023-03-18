use std::ops::Mul;

pub struct DcMotor {
  pub v_nom: f64,
  pub t_stall: f64,
  pub i_stall: f64,
  pub i_free: f64,
  pub w_free: f64
}

impl DcMotor {
  pub fn new(v_nom: f64, t_stall: f64, i_stall: f64, i_free: f64, w_free: f64) -> Self {
    Self {
      v_nom, t_stall, i_stall,
      i_free, w_free
    }
  }

  #[allow(non_snake_case)]
  pub fn R(&self) -> f64 {
    self.v_nom / self.i_stall
  }

  pub fn kw(&self) -> f64 {
    self.w_free / (self.v_nom - self.R() * self.i_free)
  }

  pub fn kt(&self) -> f64 {
    self.t_stall / self.i_stall
  }

  pub fn current(&self, speed: f64, voltage: f64) -> f64 {
    -1.0 / self.kw() / self.R() * speed + 1.0 / self.R() * voltage
  }

  pub fn torque(&self, current: f64) -> f64 {
    (current * self.kt()).into()
  }

  pub fn voltage(&self, torque: f64, speed: f64) -> f64 {
    1.0 / self.kw() * speed + 1.0 / self.kt() * self.R() * torque
  }

  pub fn speed(&self, torque: f64, voltage: f64) -> f64 {
    (voltage * self.kw() - 1.0 / self.kt() * torque * self.R() * self.kw()).into()
  }

  pub fn reduce(self, reduction: f64) -> Self {
    Self::new(self.v_nom, self.t_stall * reduction, self.i_stall, self.i_free, self.w_free / reduction)
  }
}

impl Mul<f64> for DcMotor {
  type Output = DcMotor;

  fn mul(self, rhs: f64) -> Self::Output {
    DcMotor::new(self.v_nom, self.t_stall * rhs, self.i_stall * rhs, self.i_free * rhs, self.w_free)
  }
}

macro_rules! define_motor {
  ($name:ident, $v:literal, $t_stall:literal, $i_stall:literal, $i_free: literal, $w_free:literal) => {
    #[allow(non_snake_case)]
    pub fn $name() -> Self {
      Self::new( $v, $t_stall, $i_stall, $i_free, $w_free )
    }
  }
}

impl DcMotor {
  define_motor!(CIM,          12.0,   2.42,   133.0,  2.7,  5310.0);
  define_motor!(miniCIM,      12.0,   1.41,   89.0,   3.0,  5840.0);
  define_motor!(bag,          12.0,   0.43,   53.0,   1.8,  13180.0);
  define_motor!(vex775pro,    12.0,   0.71,   134.0,  0.7,  18730.0);
  define_motor!(rs775125,     12.0,   0.28,   18.0,   1.6,  5800.0);
  define_motor!(banebots775,  12.0,   0.72,   97.0,   2.7,  13050.0);
  define_motor!(andymark9015, 12.0,   0.36,   71.0,   3.7,  14270.0);
  define_motor!(banebots550,  12.0,   0.38,   84.0,   0.4,  19000.0);
  define_motor!(neo,          12.0,   2.6,    105.0,  1.8,  5676.0);
  define_motor!(neo550,       12.0,   0.97,   100.0,  1.4,  11000.0);
  define_motor!(falcon500,    12.0,   4.69,   257.0,  1.5,  6380.0);
}