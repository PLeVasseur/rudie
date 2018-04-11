#![feature(proc_macro)]
#![no_std]

pub extern crate mat;
pub extern crate typenum;
pub extern crate generic_array;

use core::ops::{Mul};
use core::fmt;

use mat::traits::{Zero};
use typenum::{Unsigned, Prod};
use typenum::consts::*;
use generic_array::{ArrayLength};

pub struct KalmanFilter<T, DP, MP, CP>
where
    T: Copy + Zero + Default,
    DP: Unsigned,
    MP: Unsigned,
    CP: Unsigned,
    DP: Mul<U1>,
    Prod<DP, U1>: ArrayLength<T>,
    DP: Mul<DP>,
    Prod<DP, DP>: ArrayLength<T>,
    MP: Mul<DP>,
    Prod<MP, DP>: ArrayLength<T>,
    MP: Mul<MP>,
    Prod<MP, MP>: ArrayLength<T>,
    DP: Mul<MP>,
    Prod<DP, MP>: ArrayLength<T>,
    DP: Mul<CP>,
    Prod<DP, CP>: ArrayLength<T>
{
    pub state_pre: mat::MatGenImm<T, DP, U1>,
    pub state_post: mat::MatGenImm<T, DP, U1>,
    pub transition_matrix: mat::MatGenImm<T, DP, DP>,

    pub process_noise_cov: mat::MatGenImm<T, DP, DP>,
    pub measurement_matrix: mat::MatGenImm<T, MP, DP>,
    pub measurement_noise_cov: mat::MatGenImm<T, MP, MP>,

    pub error_cov_pre: mat::MatGenImm<T, DP, DP>,
    pub error_cov_post: mat::MatGenImm<T, DP, DP>,
    pub gain: mat::MatGenImm<T, DP, MP>,

    pub control_matrix: mat::MatGenImm<T, DP, CP>
}

impl<'b, 'a: 'b, T, DP, MP, CP> KalmanFilter<T, DP, MP, CP>
where
    T: Copy + Zero + Default + 'b,
    DP: Unsigned + 'b,
    MP: Unsigned,
    CP: Unsigned,
    DP: Mul<U1>,
    Prod<DP, U1>: ArrayLength<T>,
    DP: Mul<DP>,
    Prod<DP, DP>: ArrayLength<T>,
    MP: Mul<DP>,
    Prod<MP, DP>: ArrayLength<T>,
    MP: Mul<MP>,
    Prod<MP, MP>: ArrayLength<T>,
    DP: Mul<MP>,
    Prod<DP, MP>: ArrayLength<T>,
    DP: Mul<CP>,
    Prod<DP, CP>: ArrayLength<T>,
    &'b mat::MatGenImm<T, DP, DP>: Mul<&'b mat::MatGenImm<T, DP, U1>, Output = mat::MatGenImm<T, DP, U1>>,
{
    pub fn init() -> Self {
        KalmanFilter {
            state_pre: Default::default(),
            state_post: Default::default(),
            transition_matrix: Default::default(),

            process_noise_cov: Default::default(),
            measurement_matrix: Default::default(),
            measurement_noise_cov: Default::default(),

            error_cov_pre: Default::default(),
            error_cov_post: Default::default(),
            gain: Default::default(),

            control_matrix: Default::default()
        }
    }

    pub fn predict(&'a mut self/*, control: mat::MatGenImm<T, DP, CP>*/) {

        self.state_pre = &self.transition_matrix * &self.state_post;

    }
}

impl<T, DP, MP, CP> fmt::Debug for KalmanFilter<T, DP, MP, CP>
where
    T: Copy + Zero + Default + fmt::Debug,
    DP: Unsigned,
    MP: Unsigned,
    CP: Unsigned,
    DP: Mul<U1>,
    Prod<DP, U1>: ArrayLength<T>,
    DP: Mul<DP>,
    Prod<DP, DP>: ArrayLength<T>,
    MP: Mul<DP>,
    Prod<MP, DP>: ArrayLength<T>,
    MP: Mul<MP>,
    Prod<MP, MP>: ArrayLength<T>,
    DP: Mul<MP>,
    Prod<DP, MP>: ArrayLength<T>,
    DP: Mul<CP>,
    Prod<DP, CP>: ArrayLength<T>
{
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.write_str("state_pre: ")?;
        write!(f, "{:?}; ", self.state_pre)?;
        f.write_str("state_post: ")?;
        write!(f, "{:?}; ", self.state_post)?;
        f.write_str("transition_matrix: ")?;
        write!(f, "{:?}; ", self.transition_matrix)?;
        f.write_str("process_noise_cov: ")?;
        write!(f, "{:?}; ", self.process_noise_cov)?;
        f.write_str("measurement_matrix: ")?;
        write!(f, "{:?}; ", self.measurement_matrix)?;
        f.write_str("measurement_noise_cov: ")?;
        write!(f, "{:?}; ", self.measurement_noise_cov)?;
        f.write_str("error_cov_pre: ")?;
        write!(f, "{:?}; ", self.error_cov_pre)?;
        f.write_str("error_cov_post: ")?;
        write!(f, "{:?}; ", self.error_cov_post)?;
        f.write_str("gain: ")?;
        write!(f, "{:?}; ", self.gain)?;
        f.write_str("control_matrix: ")?;
        write!(f, "{:?}; ", self.control_matrix)
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {

        assert_eq!(2 + 2, 4);
    }
}