#![feature(proc_macro)]
#![no_std]

pub extern crate mat;
pub extern crate typenum;
pub extern crate generic_array;

use core::ops::{Mul, Add, Sub};
use core::fmt;

use mat::traits::{Zero, TransposeImm};
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
    Prod<DP, CP>: ArrayLength<T>,
    CP: Mul<U1>,
    Prod<CP, U1>: ArrayLength<T>,
    MP: Mul<U1>,
    Prod<MP, U1>: ArrayLength<T>,
    DP: Mul<MP>,
    Prod<DP, MP>: ArrayLength<T>,
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

    pub control_matrix: mat::MatGenImm<T, DP, CP>,

    temp1: mat::MatGenImm<T, DP, U1>,
    temp2: mat::MatGenImm<T, DP, U1>,
    temp3: mat::MatGenImm<T, CP, U1>,
    temp4: mat::MatGenImm<T, DP, DP>,
    temp5: mat::MatGenImm<T, DP, DP>,
    temp6: mat::MatGenImm<T, DP, DP>,
    temp7: mat::MatGenImm<T, MP, DP>,
    temp8: mat::MatGenImm<T, MP, MP>,
    temp9: mat::MatGenImm<T, MP, MP>,
    temp10: mat::MatGenImm<T, DP, MP>,
    temp11: mat::MatGenImm<T, MP, U1>,
    temp12: mat::MatGenImm<T, MP, U1>,
    temp13: mat::MatGenImm<T, MP, U1>,
}

impl<'b, 'a: 'b, T, DP, MP, CP> KalmanFilter<T, DP, MP, CP>
where
    T: Copy + Zero + Default + 'b,
    DP: Unsigned + 'b,
    MP: Unsigned + 'b,
    CP: Unsigned + 'b,
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
    DP: Mul<U1>,
    Prod<DP, U1>: ArrayLength<T>,
    CP: Mul<U1>,
    Prod<CP, U1>: ArrayLength<T>,
    MP: Mul<U1>,
    Prod<MP, U1>: ArrayLength<T>,
    DP: Mul<MP>,
    Prod<DP, MP>: ArrayLength<T>,
    // --- below bounds for fn predict ---
    // below bounds used for updating state_pre based on process model
    &'b mat::MatGenImm<T, DP, DP>: Mul<&'b mat::MatGenImm<T, DP, U1>, Output = mat::MatGenImm<T, DP, U1>>,
    // below bounds used for updating state_pre based on control model
    mat::MatGenImm<T, DP, U1>: core::clone::Clone,
    mat::MatGenImm<T, DP, CP>: core::clone::Clone,
    mat::MatGenImm<T, CP, U1> : core::clone::Clone,
    &'b mat::MatGenImm<T, DP, CP>: Mul<&'b mat::MatGenImm<T, CP, U1>, Output = mat::MatGenImm<T, DP, U1>>,
    &'b mat::MatGenImm<T, DP, U1>: Add<&'b mat::MatGenImm<T, DP, U1>, Output = mat::MatGenImm<T, DP, U1>>,
    // below bounds used for updating error_cov_pre
    &'b mat::MatGenImm<T, DP, DP>: Mul<&'b mat::MatGenImm<T, DP, DP>, Output = mat::MatGenImm<T, DP, DP>>,
    &'b mat::MatGenImm<T, DP, DP>: mat::traits::TransposeImm<Output = mat::MatGenImm<T, DP, DP>>,
    &'b mat::MatGenImm<T, DP, DP>: Add<&'b mat::MatGenImm<T, DP, DP>, Output = mat::MatGenImm<T, DP, DP>>,
    // error_cov_pre update
    mat::MatGenImm<T, DP, DP>: core::clone::Clone,
    // --- below bounds for fn correct --
    // finding gain
    &'b mat::MatGenImm<T, MP, DP>: Mul<&'b mat::MatGenImm<T, DP, U1>, Output = mat::MatGenImm<T, MP, U1>>,
    &'b mat::MatGenImm<T, MP, DP>: Mul<&'b mat::MatGenImm<T, DP, DP>, Output = mat::MatGenImm<T, MP, DP>>,
    &'b mat::MatGenImm<T, MP, DP>: Mul<&'b mat::MatGenImm<T, DP, MP>, Output = mat::MatGenImm<T, MP, MP>>,
    &'b mat::MatGenImm<T, MP, DP>: mat::traits::TransposeImm<Output = mat::MatGenImm<T, DP, MP>>,
    &'b mat::MatGenImm<T, MP, MP>: Add<&'b mat::MatGenImm<T, MP, MP>, Output = mat::MatGenImm<T, MP, MP>>,
    mat::MatGenImm<T, MP, U1>: core::clone::Clone,
    &'b mat::MatGenImm<T, MP, U1>: Sub<&'b mat::MatGenImm<T, MP, U1>, Output = mat::MatGenImm<T, MP, U1>>,
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

            control_matrix: Default::default(),

            temp1: Default::default(),
            temp2: Default::default(),
            temp3: Default::default(),
            temp4: Default::default(),
            temp5: Default::default(),
            temp6: Default::default(),
            temp7: Default::default(),
            temp8: Default::default(),
            temp9: Default::default(),
            temp10: Default::default(),
            temp11: Default::default(),
            temp12: Default::default(),
            temp13: Default::default()
        }
    }

    pub fn predict(&'a mut self, control: mat::MatGenImm<T, CP, U1>) -> mat::MatGenImm<T, DP, U1> {

        self.state_pre = &self.transition_matrix * &self.state_post;

        if CP::to_usize() > 0 {

            // TODO: Perhaps there's a better way to satisfy the borrow checker than clone'ing
            // we would like to avoid run-time allocations
            self.temp3 = control.clone();
            self.temp2 = &self.control_matrix * &self.temp3;
            self.temp1 = self.state_pre.clone();
            self.state_pre = &self.temp1 + &self.temp2;
        }

        self.temp4 = &self.transition_matrix * &self.error_cov_post;
        self.temp5 = self.transition_matrix.t();
        self.temp6 = &self.temp4 * &self.temp5;
        self.error_cov_pre = &self.temp6 + &self.process_noise_cov;

        self.state_pre = self.state_post.clone();
        self.error_cov_pre = self.error_cov_post.clone();

        self.state_pre.clone()
    }


    pub fn correct(&'a mut self, measurement: mat::MatGenImm<T, MP, U1>) {

        self.temp12 = measurement.clone();
        // residual calculation
        self.temp11 = &self.measurement_matrix * &self.state_pre;
        self.temp13 = &self.temp12 - &self.temp11; // y

        // innovation covariance calculation
        self.temp7 = &self.measurement_matrix * &self.error_cov_pre;
        self.temp10 = self.measurement_matrix.t();
        self.temp8 = &self.temp7 * &self.temp10;
        self.temp9 = &self.temp8 + &self.measurement_noise_cov; // S


        // K
    }
}

impl<T, DP, MP, CP> fmt::Debug for KalmanFilter<T, DP, MP, CP>
where
    T: Copy + Zero + Default + fmt::Debug,
    DP: Unsigned,
    MP: Unsigned,
    CP: Unsigned,

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
    CP: Mul<U1>,
    Prod<CP, U1>: ArrayLength<T>,
    DP: Mul<U1>,
    Prod<DP, U1>: ArrayLength<T>,
    MP: Mul<U1>,
    Prod<MP, U1>: ArrayLength<T>,
    DP: Mul<MP>,
    Prod<DP, MP>: ArrayLength<T>,
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
        write!(f, "{:?}; ", self.control_matrix)?;
        f.write_str("temp1: ")?;
        write!(f, "{:?}; ", self.temp1)?;
        f.write_str("temp2: ")?;
        write!(f, "{:?}; ", self.temp2)?;
        f.write_str("temp3: ")?;
        write!(f, "{:?}; ", self.temp3)
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {

        assert_eq!(2 + 2, 4);
    }
}