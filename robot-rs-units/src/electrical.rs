use typenum::{N3, P2, P1, N1, Z0, N2, P4};

use super::{Unit, Quantity, ISQ, QuantityBase};
use crate::unit;

pub type Voltage      = Quantity<ISQ<N3, P2, P1, N1, Z0, Z0, Z0, Z0, Z0>>;
pub type Resistance   = Quantity<ISQ<N3, P2, P1, N2, Z0, Z0, Z0, Z0, Z0>>;
pub type Capacitance  = Quantity<ISQ<P4, N2, N1, P2, Z0, Z0, Z0, Z0, Z0>>;
pub type Inductance   = Quantity<ISQ<N2, P2, P1, N2, Z0, Z0, Z0, Z0, Z0>>;

unit!(Voltage, volt, 1.0, 0.0);
unit!(Voltage, millivolt, 1e-3, 0.0);
unit!(Voltage, microvolt, 1e-6, 0.0);

unit!(Resistance, ohm, 1.0, 0.0);
unit!(Resistance, milliohm, 1e-3, 0.0);
unit!(Resistance, kiloohm, 1e3, 0.0);

unit!(Capacitance, farad, 1.0, 0.0);
unit!(Capacitance, millifarad, 1e-3, 0.0);
unit!(Capacitance, microfarad, 1e-6, 0.0);
unit!(Capacitance, nanofarad, 1e-9, 0.0);

unit!(Inductance, henry, 1.0, 0.0);
unit!(Inductance, microhenry, 1e-3, 0.0);
unit!(Inductance, millihenry, 1e-6, 0.0);
unit!(Inductance, nanohenry, 1e-9, 0.0);
