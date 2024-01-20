
use typenum::{N2, P1, Z0, P2};

use super::{Unit, Quantity, ISQ, QuantityBase};
use crate::unit;

pub type Force = Quantity<ISQ<N2, P1, P1, Z0, Z0, Z0, Z0, Z0, Z0>>;
pub type Torque = Quantity<ISQ<N2, P2, P1, Z0, Z0, Z0, Z0, Z0, Z0>>;
pub type MOI = Quantity<ISQ<Z0, P2, P1, Z0, Z0, Z0, Z0, Z0, Z0>>;

unit!(Force, newton, 1.0, 0.0);

unit!(Torque, newton_meter, 1.0, 0.0);

unit!(MOI, kgm2, 1.0, 1.0);