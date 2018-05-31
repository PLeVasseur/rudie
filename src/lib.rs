#![feature(proc_macro)]
#![no_std]

pub extern crate typenum;
pub extern crate generic_array;
pub extern crate nalgebra as na;

use core::ops::{Mul, Sub};
use core::fmt;

use generic_array::{ArrayLength};

use na::{U1, Matrix, MatrixArray, DimName, Vector, zero, SVD};

const FLT_EPSILON: f64 = 1.19209290e-07;

pub struct KalmanFilter<DP, MP, CP>
where
//    T: Copy + PartialEq + Debug,
    DP: DimName,
    MP: DimName,
    CP: DimName,
    <DP as DimName>::Value: Mul<typenum::U1>,
    <<DP as DimName>::Value as Mul<typenum::U1>>::Output: ArrayLength<f64>,
    <DP as DimName>::Value: Mul,
    <<DP as DimName>::Value as Mul>::Output: ArrayLength<f64>,
    <MP as DimName>::Value: Mul<<DP as DimName>::Value>,
    <<MP as DimName>::Value as Mul<<DP as DimName>::Value>>::Output: ArrayLength<f64>,
    <MP as DimName>::Value: Mul,
    <<MP as DimName>::Value as Mul>::Output: ArrayLength<f64>,
    <DP as DimName>::Value: Mul<<CP as DimName>::Value>,
    <<DP as DimName>::Value as Mul<<CP as DimName>::Value>>::Output: ArrayLength<f64>,
    <DP as DimName>::Value: Mul<<MP as DimName>::Value>,
    <<DP as DimName>::Value as Mul<<MP as DimName>::Value>>::Output: ArrayLength<f64>,
    <MP as DimName>::Value: Mul<typenum::U1>,
    <<MP as DimName>::Value as Mul<typenum::U1>>::Output: ArrayLength<f64>
{
    pub state_pre: Vector<f64, DP, MatrixArray<f64, DP, U1>>,
    pub state_post: Vector<f64, DP, MatrixArray<f64, DP, U1>>,
    pub transition_matrix: Matrix<f64, DP, DP, MatrixArray<f64, DP, DP>>,

    pub process_noise_cov: Matrix<f64, DP, DP, MatrixArray<f64, DP, DP>>,
    pub measurement_matrix: Matrix<f64, MP, DP, MatrixArray<f64, MP, DP>>,
    pub measurement_noise_cov: Matrix<f64, MP, MP, MatrixArray<f64, MP, MP>>,

    pub control_matrix: Matrix<f64, DP, CP, MatrixArray<f64, DP, CP>>,

    pub error_cov_pre: Matrix<f64, DP, DP, MatrixArray<f64, DP, DP>>,
    pub error_cov_post: Matrix<f64, DP, DP, MatrixArray<f64, DP, DP>>,
    pub gain: Matrix<f64, DP, MP, MatrixArray<f64, DP, MP>>,

    pub residual: Vector<f64, MP, MatrixArray<f64, MP, U1>>,
    pub innov_cov: Matrix<f64, MP, MP, MatrixArray<f64, MP, MP>>
}

impl<DP, MP, CP> KalmanFilter<DP, MP, CP>
where
//    T: Copy + PartialEq + Debug,
    DP: DimName,
    MP: DimName,
    CP: DimName,
    <DP as DimName>::Value: Mul<typenum::U1>,
    <<DP as DimName>::Value as Mul<typenum::U1>>::Output: ArrayLength<f64>,
    <DP as DimName>::Value: Mul,
    <<DP as DimName>::Value as Mul>::Output: ArrayLength<f64>,
    <MP as DimName>::Value: Mul<<DP as DimName>::Value>,
    <<MP as DimName>::Value as Mul<<DP as DimName>::Value>>::Output: ArrayLength<f64>,
    <MP as DimName>::Value: Mul,
    <<MP as DimName>::Value as Mul>::Output: ArrayLength<f64>,
    <DP as DimName>::Value: Mul<<CP as DimName>::Value>,
    <<DP as DimName>::Value as Mul<<CP as DimName>::Value>>::Output: ArrayLength<f64>,
    <DP as DimName>::Value: Mul<<MP as DimName>::Value>,
    <<DP as DimName>::Value as Mul<<MP as DimName>::Value>>::Output: ArrayLength<f64>,
    <MP as DimName>::Value: Mul<typenum::U1>,
    <<MP as DimName>::Value as Mul<typenum::U1>>::Output: ArrayLength<f64>,
    // fn predict
    <CP as DimName>::Value: Mul<typenum::U1>,
    <<CP as DimName>::Value as Mul<typenum::U1>>::Output: ArrayLength<f64>,
    // fn correct
    <MP as DimName>::Value: Mul<typenum::U1>,
    <<MP as DimName>::Value as Mul<typenum::U1>>::Output: ArrayLength<f64>,
    <MP as na::DimName>::Value: typenum::Min,
    <<MP as DimName>::Value as typenum::Min>::Output: na::NamedDim,
    <<MP as DimName>::Value as typenum::Min>::Output: Mul<<MP as DimName>::Value>,
    <<<MP as DimName>::Value as typenum::Min>::Output as Mul<<MP as DimName>::Value>>::Output: ArrayLength<f64>,
    <MP as DimName>::Value: Mul<<<MP as DimName>::Value as typenum::Min>::Output>,
    <<MP as DimName>::Value as Mul<<<MP as DimName>::Value as typenum::Min>::Output>>::Output: ArrayLength<f64>,
    <<MP as DimName>::Value as typenum::Min>::Output: Mul<typenum::U1>,
    <<<MP as DimName>::Value as typenum::Min>::Output as Mul<typenum::U1>>::Output: ArrayLength<f64>,
    <<MP as DimName>::Value as typenum::Min>::Output: Sub<typenum::U1>,
    <<<MP as DimName>::Value as typenum::Min>::Output as Sub<typenum::U1>>::Output: na::NamedDim,
    <<<MP as DimName>::Value as typenum::Min>::Output as Sub<typenum::U1>>::Output: Mul<typenum::U1>,
    <<<<MP as DimName>::Value as typenum::Min>::Output as Sub<typenum::U1>>::Output as Mul<typenum::U1>>::Output: ArrayLength<f64>,
    <<MP as DimName>::Value as typenum::Min>::Output: Mul<<DP as DimName>::Value>,
    <<<MP as DimName>::Value as typenum::Min>::Output as Mul<<DP as DimName>::Value>>::Output: ArrayLength<f64>
{
    pub fn init() -> Self {
        KalmanFilter {
            state_pre: zero(),
            state_post: zero(),
            transition_matrix: zero(),

            process_noise_cov: zero(),
            measurement_matrix: zero(),
            measurement_noise_cov: zero(),

            error_cov_pre: zero(),
            error_cov_post: zero(),
            gain: zero(),

            control_matrix: zero(),

            residual: zero(),
            innov_cov: zero()
        }
    }

    pub fn predict(&mut self, control: Vector<f64, CP, MatrixArray<f64, CP, U1>>)
        -> Vector<f64, DP, MatrixArray<f64, DP, U1>>
    {

        // x'(k) = A*x(k)
        self.state_pre = &self.transition_matrix * &self.state_post;

        if CP::dim() > 0 {
            // x'(k) = x'(k) + B*u(k)
            self.state_pre = &self.state_pre + &self.control_matrix * control;
        }

        // P'(k) = A*P(k)*At + Q
        self.error_cov_pre = &self.transition_matrix * &self.error_cov_post * self.transition_matrix.transpose() + &self.process_noise_cov;

        // handle the case when there will be measurement before the next predict.
        self.state_post = self.state_pre.clone();
        self.error_cov_post = self.error_cov_pre.clone();


        self.state_pre.clone()
    }

    pub fn correct(&mut self, measurement: Vector<f64, MP, MatrixArray<f64, MP, U1>>)
                   -> Vector<f64, DP, MatrixArray<f64, DP, U1>>
    {

        // y(k) = z(k) - H*x'(k)
        self.residual = measurement - (&self.measurement_matrix * &self.state_pre); // y

        // S(k) = H*x'(k)*Ht + R
        self.innov_cov = &self.measurement_matrix * &self.error_cov_pre * self.measurement_matrix.transpose() + &self.measurement_noise_cov;

        // temp1 = H*P'(k)
        let temp1 = &self.measurement_matrix * &self.error_cov_pre;


        let svd = SVD::new(self.innov_cov.clone(), true, true);

        // temp2 = inv(S)*temp1 = Kt(k)
        let temp2 = svd.solve(&temp1, FLT_EPSILON * 2.0);

        // K(k)
        self.gain = temp2.transpose();

        // x(k) = x'(k) + K(k)*y(k)
        self.state_post = &self.state_pre + &self.gain*&self.residual;

        // P(k) = P'(k) - K(k)*temp1
        self.error_cov_post = &self.error_cov_pre - &self.gain*temp1;


        self.state_post.clone()
    }
}

impl<DP, MP, CP> fmt::Debug for KalmanFilter<DP, MP, CP>
where
//    T: Copy + Zero + Default + fmt::Debug,
    DP: DimName,
    MP: DimName,
    CP: DimName,
    <DP as DimName>::Value: Mul<typenum::U1>,
    <<DP as DimName>::Value as Mul<typenum::U1>>::Output: ArrayLength<f64>,
    <DP as DimName>::Value: Mul,
    <<DP as DimName>::Value as Mul>::Output: ArrayLength<f64>,
    <MP as DimName>::Value: Mul<<DP as DimName>::Value>,
    <<MP as DimName>::Value as Mul<<DP as DimName>::Value>>::Output: ArrayLength<f64>,
    <MP as DimName>::Value: Mul,
    <<MP as DimName>::Value as Mul>::Output: ArrayLength<f64>,
    <DP as DimName>::Value: Mul<<CP as DimName>::Value>,
    <<DP as DimName>::Value as Mul<<CP as DimName>::Value>>::Output: ArrayLength<f64>,
    <DP as DimName>::Value: Mul<<MP as DimName>::Value>,
    <<DP as DimName>::Value as Mul<<MP as DimName>::Value>>::Output: ArrayLength<f64>,
    <MP as DimName>::Value: Mul<typenum::U1>,
    <<MP as DimName>::Value as Mul<typenum::U1>>::Output: ArrayLength<f64>
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
        f.write_str("residual: ")?;
        write!(f, "{:?}; ", self.residual)?;
        f.write_str("innov_cov: ")?;
        write!(f, "{:?}; ", self.innov_cov)
    }
}
