#![no_std]

/*!
# rudie

**rudie** has a Kalman Filter implementation that will work on embedded platforms for robotics
applications.

## Using **rudie**
You will need the last stable build of the [rust compiler](http://www.rust-lang.org)
and the official package manager: [cargo](https://github.com/rust-lang/cargo).

Simply add the following to your `Cargo.toml` file:

```.ignore
[dependencies]
rudie = "0.1"
```

## Features
**rudie** is meant to collect together Kalman filter implementations that may be used on embedded
devices for robotics applications since it is designed to run on `#![no_std]` targets.
Those features include:

* An implementation of the OpenCV Kalman filter that will run on `#![no_std]` targets
*/

pub extern crate typenum;
pub extern crate generic_array;
pub extern crate nalgebra as na;

use core::ops::{Mul, Sub};

/// A Rust implementation of the OpenCV Kalman Filter
///
/// Because Rust currently doesn't have const generics, we make heavy use of the typenum
/// to specify type-level numerics that are used when interacting with the library.
///
/// rudie can be used in `#![no_std]` mode for embedded applications with no OS since we make use
/// of only `#![no_std]` compatible libraries
///
/// # Examples
///
/// The following example shows a Kalman filter estimating the orientation and rotational rate
/// of a rotating point. An example is given further down showing that the kalman filter is correct
/// by examining its internal state.
///
/// ```
/// extern crate rudie;
/// extern crate rand;
/// extern crate libm;
/// extern crate assert as asrt;
///
/// use rand::{Rng, StdRng, SeedableRng};
/// use rand::distributions::{Distribution, Normal};
/// use libm::F64Ext;
/// use asrt::close;
///
/// use rudie::OpenCVKalmanFilter;
/// use rudie::na::{U0, U1, U2, Vector, Matrix, ArrayStorage};
///
/// // seed the rng so we get reproducible results
/// let seed: [u8; 32] = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
///                      1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
///                      1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
///                      1, 2];
/// let mut state_rng: StdRng = SeedableRng::from_seed(seed);
/// let state_generator = Normal::new(0., 0.1);
///
/// // We use the typenum crate here to specify type-level numbers such as U0 = 0, U1 = 1, ...
/// // until Rust has support for const generics
/// let mut kf: OpenCVKalmanFilter<f64, U2, U1, U0> = OpenCVKalmanFilter::init();
///
/// // (phi, delta_phi), i.e. orientation and angular rate
/// let mut state: Vector<f64, U2, ArrayStorage<f64, U2, U1>>;
///
/// let mut process_noise: Vector<f64, U2, ArrayStorage<f64, U2, U1>>;
///
/// let mut measurement: Vector<f64, U1, ArrayStorage<f64, U1, U1>>;
///
/// state = rudie::na::Matrix2x1::new(
///     state_generator.sample(&mut state_rng),
///     state_generator.sample(&mut state_rng)
/// );
/// kf.transition_matrix = rudie::na::Matrix2::new(
///     1., 1.,
///     0., 1.
/// );
///
/// kf.measurement_matrix = rudie::na::Matrix1x2::identity();
/// kf.process_noise_cov = rudie::na::Matrix2::from_diagonal_element(1e-5);
/// kf.measurement_noise_cov = rudie::na::Matrix1::from_diagonal_element(1e-1);
/// kf.error_cov_post = rudie::na::Matrix2::from_diagonal_element(1.);
///
/// let measurement_noise_generator = Normal::new(0., kf.measurement_noise_cov[(0)]);
/// let mut measurement_noise_rng: StdRng = SeedableRng::from_seed(seed);
/// let process_noise_generator = Normal::new(0., kf.process_noise_cov[(0)].sqrt());
/// let mut process_noise_rng: StdRng = SeedableRng::from_seed(seed);
///
/// // set up the initial state
/// kf.state_post = rudie::na::Matrix2x1::new(
///     -0.13210335109501573,
///     -0.06167960574363984
/// );
///
/// for cycle in 0..3 {
///     // Kalman predict
///     kf.predict_no_control();
///
///     // generate measurement
///     measurement = rudie::na::Matrix1::new(
///         measurement_noise_generator.sample(&mut measurement_noise_rng)
///     );
///     measurement += kf.measurement_matrix*state;
///
///     // Kalman correct
///     kf.correct(measurement);
///
///     // generate next state
///     process_noise = rudie::na::Matrix2x1::new(
///         process_noise_generator.sample(&mut process_noise_rng),
///         process_noise_generator.sample(&mut process_noise_rng)
///     );
///     state = kf.transition_matrix * state + process_noise;
/// }
///
/// ```
///
/// The following example shows a Kalman filter estimating the orientation and rotational rate
/// of a rotating point. Assertions are included to ensure the example code does not regress.
///
/// ```
/// extern crate rudie;
/// extern crate rand;
/// extern crate libm;
/// extern crate assert as asrt;
///
/// use rand::{Rng, StdRng, SeedableRng};
/// use rand::distributions::{Distribution, Normal};
/// use libm::F64Ext;
/// use asrt::close;
///
/// use rudie::OpenCVKalmanFilter;
/// use rudie::na::{U0, U1, U2, Vector, Matrix, ArrayStorage};
///
/// /**************************************************
/// ** filter configurations to assert against - begin
/// ***************************************************/
///
/// // known filter configurations to assert state against for kalman predict
/// let state_pre_assert_predict = rudie::na::Matrix2x3::new(
///     -0.19378295683865557, 0.1361050053334287,   0.04097848073026808,
///     -0.06167960574363984, 0.06884248182130964, -0.00403224753640588
/// );
/// let error_cov_pre_assert_predict = rudie::na::Matrix2x6::new(
///     2.00001, 1.,      0.7143075510116619, 0.57144061223518,   0.3509028377933237, 0.1929981748186918,
///     1.,      1.00001, 0.57144061223518,   0.5238317913724221, 0.1929981748186918, 0.1228331394128128
/// );
///
/// // known filter configurations to assert state against for kalman correct
/// let residual_assert_correct = rudie::na::Matrix1x3::new(
///     0.27409768910726956, -0.10384708598466963, -0.1451463671400915
/// );
/// let innov_cov_assert_correct = rudie::na::Matrix1x3::new(
///     2.10001, 0.8143075510116619, 0.45090283779332374
/// );
/// let gain_assert_correct = rudie::na::Matrix2x3::new(
///     0.9523811791372422, 0.8771962756875286, 0.7782227308894513,
///     0.47618820862757794, 0.70175035405879,  0.4280260815460971
/// );
/// let state_post_assert_correct = rudie::na::Matrix2x3::new(
///     0.06726252351211906, 0.04501072826667396,  -0.07197772148417683,
///     0.06884248182130964, -0.00403224753640588, -0.06615867831403044
/// );
/// let error_cov_post_assert_correct = rudie::na::Matrix2x6::new(
///     0.09523811791372422, 0.04761882086275779, 0.08771962756875286, 0.07017503540587901, 0.07782227308894514, 0.04280260815460972,
///     0.04761882086275779, 0.5238217913724221,  0.07017503540587901, 0.12282313941281281, 0.04280260815460972, 0.04022488689961951
/// );
///
/// /**************************************************
/// ** filter configurations to assert against - end
/// ***************************************************/
///
/// // seed the rng so we get reproducible results
/// let seed: [u8; 32] = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
///                      1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
///                      1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
///                      1, 2];
/// let mut state_rng: StdRng = SeedableRng::from_seed(seed);
/// let state_generator = Normal::new(0., 0.1);
///
/// // We use the typenum crate here to specify type-level numbers such as U0 = 0, U1 = 1, ...
/// // until Rust has support for const generics
/// let mut kf: OpenCVKalmanFilter<f64, U2, U1, U0> = OpenCVKalmanFilter::init();
///
/// // (phi, delta_phi), i.e. orientation and angular rate
/// let mut state: Vector<f64, U2, ArrayStorage<f64, U2, U1>>;
///
/// let mut process_noise: Vector<f64, U2, ArrayStorage<f64, U2, U1>>;
///
/// let mut measurement: Vector<f64, U1, ArrayStorage<f64, U1, U1>>;
///
/// state = rudie::na::Matrix2x1::new(
///     state_generator.sample(&mut state_rng),
///     state_generator.sample(&mut state_rng)
/// );
/// kf.transition_matrix = rudie::na::Matrix2::new(
///     1., 1.,
///     0., 1.
/// );
///
/// kf.measurement_matrix = rudie::na::Matrix1x2::identity();
/// kf.process_noise_cov = rudie::na::Matrix2::from_diagonal_element(1e-5);
/// kf.measurement_noise_cov = rudie::na::Matrix1::from_diagonal_element(1e-1);
/// kf.error_cov_post = rudie::na::Matrix2::from_diagonal_element(1.);
///
/// let measurement_noise_generator = Normal::new(0., kf.measurement_noise_cov[(0)]);
/// let mut measurement_noise_rng: StdRng = SeedableRng::from_seed(seed);
/// let process_noise_generator = Normal::new(0., kf.process_noise_cov[(0)].sqrt());
/// let mut process_noise_rng: StdRng = SeedableRng::from_seed(seed);
///
/// // set up the initial state
/// kf.state_post = rudie::na::Matrix2x1::new(
///     -0.13210335109501573,
///     -0.06167960574363984
/// );
///
/// for cycle in 0..3 {
///     // Kalman predict
///     kf.predict_no_control();
///
///     /********************************************************************
///     ** assert known filter configuration after predict_no_control - begin
///     *********************************************************************/
///     close(kf.state_pre[(0,0)], state_pre_assert_predict[(0,cycle)], core::f64::EPSILON);
///     close(kf.state_pre[(1,0)], state_pre_assert_predict[(1,cycle)], core::f64::EPSILON);
///
///     close(kf.error_cov_pre[(0,0)], error_cov_pre_assert_predict[(0,cycle*2)], core::f64::EPSILON);
///     close(kf.error_cov_pre[(1,0)], error_cov_pre_assert_predict[(1,cycle*2)], core::f64::EPSILON);
///     close(kf.error_cov_pre[(0,1)], error_cov_pre_assert_predict[(0,cycle*2+1)], core::f64::EPSILON);
///     close(kf.error_cov_pre[(1,1)], error_cov_pre_assert_predict[(1,cycle*2+1)], core::f64::EPSILON);
///     /********************************************************************
///     ** assert known filter configuration after predict_no_control - end
///     *********************************************************************/
///
///     // generate measurement
///     measurement = rudie::na::Matrix1::new(
///         measurement_noise_generator.sample(&mut measurement_noise_rng)
///     );
///     measurement += kf.measurement_matrix*state;
///
///     // Kalman correct
///     kf.correct(measurement);
///
///     /*************************************************************
///     ** assert known filter configuration after correct - begin
///     **************************************************************/
///     close(kf.residual[(0,0)], residual_assert_correct[(0, cycle)], core::f64::EPSILON);
///
///     close(kf.innov_cov[(0, 0)], innov_cov_assert_correct[(0, cycle)], core::f64::EPSILON);
///
///     close(kf.gain[(0, 0)], gain_assert_correct[(0, cycle)], core::f64::EPSILON);
///     close(kf.gain[(1, 0)], gain_assert_correct[(1, cycle)], core::f64::EPSILON);
///
///     close(kf.state_post[(0,0)], state_post_assert_correct[(0, cycle)], core::f64::EPSILON);
///     close(kf.state_post[(1,0)], state_post_assert_correct[(1, cycle)], core::f64::EPSILON);
///
///     close(kf.error_cov_post[(0,0)], error_cov_post_assert_correct[(0, cycle*2)], core::f64::EPSILON);
///     close(kf.error_cov_post[(1,0)], error_cov_post_assert_correct[(1, cycle*2)], core::f64::EPSILON);
///     close(kf.error_cov_post[(0,1)], error_cov_post_assert_correct[(0, cycle*2+1)], core::f64::EPSILON);
///     close(kf.error_cov_post[(1,1)], error_cov_post_assert_correct[(1, cycle*2+1)], core::f64::EPSILON);
///     /*************************************************************
///     ** assert known filter configuration after correct - end
///     **************************************************************/
///
///     // generate next state
///     process_noise = rudie::na::Matrix2x1::new(
///         process_noise_generator.sample(&mut process_noise_rng),
///         process_noise_generator.sample(&mut process_noise_rng)
///     );
///     state = kf.transition_matrix * state + process_noise;
/// }
///
/// ```
///
pub struct OpenCVKalmanFilter<N, DP, MP, CP>
    where
        N: Real,
        DP: DimName,
        MP: DimName,
        CP: DimName,
        <DP as DimName>::Value: Mul<typenum::U1>,
        <<DP as DimName>::Value as Mul<typenum::U1>>::Output: ArrayLength<N>,
        <DP as DimName>::Value: Mul,
        <<DP as DimName>::Value as Mul>::Output: ArrayLength<N>,
        <MP as DimName>::Value: Mul<<DP as DimName>::Value>,
        <<MP as DimName>::Value as Mul<<DP as DimName>::Value>>::Output: ArrayLength<N>,
        <MP as DimName>::Value: Mul,
        <<MP as DimName>::Value as Mul>::Output: ArrayLength<N>,
        <DP as DimName>::Value: Mul<<CP as DimName>::Value>,
        <<DP as DimName>::Value as Mul<<CP as DimName>::Value>>::Output: ArrayLength<N>,
        <DP as DimName>::Value: Mul<<MP as DimName>::Value>,
        <<DP as DimName>::Value as Mul<<MP as DimName>::Value>>::Output: ArrayLength<N>,
        <MP as DimName>::Value: Mul<typenum::U1>,
        <<MP as DimName>::Value as Mul<typenum::U1>>::Output: ArrayLength<N>
{
    pub state_pre: Vector<N, DP, ArrayStorage<N, DP, U1>>,
    pub state_post: Vector<N, DP, ArrayStorage<N, DP, U1>>,
    pub transition_matrix: Matrix<N, DP, DP, ArrayStorage<N, DP, DP>>,

    pub process_noise_cov: Matrix<N, DP, DP, ArrayStorage<N, DP, DP>>,
    pub measurement_matrix: Matrix<N, MP, DP, ArrayStorage<N, MP, DP>>,
    pub measurement_noise_cov: Matrix<N, MP, MP, ArrayStorage<N, MP, MP>>,

    pub control_matrix: Matrix<N, DP, CP, ArrayStorage<N, DP, CP>>,

    pub error_cov_pre: Matrix<N, DP, DP, ArrayStorage<N, DP, DP>>,
    pub error_cov_post: Matrix<N, DP, DP, ArrayStorage<N, DP, DP>>,
    pub gain: Matrix<N, DP, MP, ArrayStorage<N, DP, MP>>,

    pub residual: Vector<N, MP, ArrayStorage<N, MP, U1>>,
    pub innov_cov: Matrix<N, MP, MP, ArrayStorage<N, MP, MP>>,
}

use core::fmt;

use generic_array::ArrayLength;

use na::{U1, Matrix, ArrayStorage, DimName, Vector, zero, SVD, Real};

impl<N, DP, MP, CP> OpenCVKalmanFilter<N, DP, MP, CP>
    where
        N: Real,
        DP: DimName,
        MP: DimName,
        CP: DimName,
        <DP as DimName>::Value: Mul<typenum::U1>,
        <<DP as DimName>::Value as Mul<typenum::U1>>::Output: ArrayLength<N>,
        <DP as DimName>::Value: Mul,
        <<DP as DimName>::Value as Mul>::Output: ArrayLength<N>,
        <MP as DimName>::Value: Mul<<DP as DimName>::Value>,
        <<MP as DimName>::Value as Mul<<DP as DimName>::Value>>::Output: ArrayLength<N>,
        <MP as DimName>::Value: Mul,
        <<MP as DimName>::Value as Mul>::Output: ArrayLength<N>,
        <DP as DimName>::Value: Mul<<CP as DimName>::Value>,
        <<DP as DimName>::Value as Mul<<CP as DimName>::Value>>::Output: ArrayLength<N>,
        <DP as DimName>::Value: Mul<<MP as DimName>::Value>,
        <<DP as DimName>::Value as Mul<<MP as DimName>::Value>>::Output: ArrayLength<N>,
        <MP as DimName>::Value: Mul<typenum::U1>,
        <<MP as DimName>::Value as Mul<typenum::U1>>::Output: ArrayLength<N>,
    // fn predict
        <CP as DimName>::Value: Mul<typenum::U1>,
        <<CP as DimName>::Value as Mul<typenum::U1>>::Output: ArrayLength<N>,
    // fn correct
        <MP as DimName>::Value: Mul<typenum::U1>,
        <<MP as DimName>::Value as Mul<typenum::U1>>::Output: ArrayLength<N>,
        <MP as na::DimName>::Value: typenum::Min,
        <<MP as DimName>::Value as typenum::Min>::Output: na::NamedDim,
        <<MP as DimName>::Value as typenum::Min>::Output: Mul<<MP as DimName>::Value>,
        <<<MP as DimName>::Value as typenum::Min>::Output as Mul<<MP as DimName>::Value>>::Output: ArrayLength<N>,
        <MP as DimName>::Value: Mul<<<MP as DimName>::Value as typenum::Min>::Output>,
        <<MP as DimName>::Value as Mul<<<MP as DimName>::Value as typenum::Min>::Output>>::Output: ArrayLength<N>,
        <<MP as DimName>::Value as typenum::Min>::Output: Mul<typenum::U1>,
        <<<MP as DimName>::Value as typenum::Min>::Output as Mul<typenum::U1>>::Output: ArrayLength<N>,
        <<MP as DimName>::Value as typenum::Min>::Output: Sub<typenum::U1>,
        <<<MP as DimName>::Value as typenum::Min>::Output as Sub<typenum::U1>>::Output: na::NamedDim,
        <<<MP as DimName>::Value as typenum::Min>::Output as Sub<typenum::U1>>::Output: Mul<typenum::U1>,
        <<<<MP as DimName>::Value as typenum::Min>::Output as Sub<typenum::U1>>::Output as Mul<typenum::U1>>::Output: ArrayLength<N>,
        <<MP as DimName>::Value as typenum::Min>::Output: Mul<<DP as DimName>::Value>,
        <<<MP as DimName>::Value as typenum::Min>::Output as Mul<<DP as DimName>::Value>>::Output: ArrayLength<N>
{
    pub fn init() -> Self {
        OpenCVKalmanFilter {
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
            innov_cov: zero(),
        }
    }

    pub fn predict(&mut self, control: Vector<N, CP, ArrayStorage<N, CP, U1>>)
                   -> Vector<N, DP, ArrayStorage<N, DP, U1>>
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

    pub fn predict_no_control(&mut self)
                              -> Vector<N, DP, ArrayStorage<N, DP, U1>>
    {
        let dummy_control: Vector<N, CP, ArrayStorage<N, CP, U1>> = zero();
        self.predict(dummy_control)
    }

    pub fn correct(&mut self, measurement: Vector<N, MP, ArrayStorage<N, MP, U1>>)
                   -> Vector<N, DP, ArrayStorage<N, DP, U1>>
    {

        // y(k) = z(k) - H*x'(k)
        self.residual = measurement - (&self.measurement_matrix * &self.state_pre); // y

        // S(k) = H*x'(k)*Ht + R
        self.innov_cov = &self.measurement_matrix * &self.error_cov_pre * self.measurement_matrix.transpose() + &self.measurement_noise_cov;

        // temp1 = H*P'(k)
        let temp1 = &self.measurement_matrix * &self.error_cov_pre;


        let svd = SVD::new(self.innov_cov.clone(), true, true);

        // temp2 = inv(S)*temp1 = Kt(k)
        let temp2 = svd.solve(&temp1, N::default_epsilon());

        // K(k)
        self.gain = temp2.transpose();

        // x(k) = x'(k) + K(k)*y(k)
        self.state_post = &self.state_pre + &self.gain * &self.residual;

        // P(k) = P'(k) - K(k)*temp1
        self.error_cov_post = &self.error_cov_pre - &self.gain * temp1;


        self.state_post.clone()
    }
}

impl<N, DP, MP, CP> fmt::Debug for OpenCVKalmanFilter<N, DP, MP, CP>
    where
        N: Real,
        DP: DimName,
        MP: DimName,
        CP: DimName,
        <DP as DimName>::Value: Mul<typenum::U1>,
        <<DP as DimName>::Value as Mul<typenum::U1>>::Output: ArrayLength<N>,
        <DP as DimName>::Value: Mul,
        <<DP as DimName>::Value as Mul>::Output: ArrayLength<N>,
        <MP as DimName>::Value: Mul<<DP as DimName>::Value>,
        <<MP as DimName>::Value as Mul<<DP as DimName>::Value>>::Output: ArrayLength<N>,
        <MP as DimName>::Value: Mul,
        <<MP as DimName>::Value as Mul>::Output: ArrayLength<N>,
        <DP as DimName>::Value: Mul<<CP as DimName>::Value>,
        <<DP as DimName>::Value as Mul<<CP as DimName>::Value>>::Output: ArrayLength<N>,
        <DP as DimName>::Value: Mul<<MP as DimName>::Value>,
        <<DP as DimName>::Value as Mul<<MP as DimName>::Value>>::Output: ArrayLength<N>,
        <MP as DimName>::Value: Mul<typenum::U1>,
        <<MP as DimName>::Value as Mul<typenum::U1>>::Output: ArrayLength<N>
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

pub trait KalmanState
where
    <Self as KalmanState>::StateLength: DimName,
    <<Self as KalmanState>::StateLength as DimName>::Value: Mul<typenum::U1>,
    <<<Self as KalmanState>::StateLength as DimName>::Value as Mul<typenum::U1>>::Output: ArrayLength<<Self as KalmanState>::FloatType>,
    <Self as KalmanState>::FloatType: Real
{
    // useful to tell the length of the state vector when we construct a filter
    type StateLength;
    type FloatType;

    fn x(&self) -> &Vector<Self::FloatType, Self::StateLength, ArrayStorage<Self::FloatType, Self::StateLength, U1>>;
}

pub trait ControlInput
where
    <Self as ControlInput>::ControlLength: DimName,
    <<Self as ControlInput>::ControlLength as DimName>::Value: Mul<typenum::U1>,
    <<<Self as ControlInput>::ControlLength as DimName>::Value as Mul<typenum::U1>>::Output: ArrayLength<<Self as ControlInput>::FloatType>,
    <Self as ControlInput>::FloatType: Real
{
    // useful to tell the length of the control vector when we construct a filter
    type ControlLength;
    type FloatType;

    fn u(&self) -> &Vector<Self::FloatType, Self::ControlLength, ArrayStorage<Self::FloatType, Self::ControlLength, U1>>;
}

pub trait SystemModel
where
    <<<Self as SystemModel>::StateType as KalmanState>::StateLength as DimName>::Value: Mul<typenum::U1>,
    <<<<Self as SystemModel>::StateType as KalmanState>::StateLength as DimName>::Value as Mul<typenum::U1>>::Output: ArrayLength<<<Self as SystemModel>::StateType as KalmanState>::FloatType>,
    <<<Self as SystemModel>::ControlType as ControlInput>::ControlLength as DimName>::Value: Mul<typenum::U1>,
    <<<<Self as SystemModel>::ControlType as ControlInput>::ControlLength as DimName>::Value as Mul<typenum::U1>>::Output: ArrayLength<<<Self as SystemModel>::ControlType as ControlInput>::FloatType>
{
    type StateType: KalmanState;
    type ControlType: ControlInput;

    // definition of state transition function
    fn f(&self, state: &Self::StateType, control: Self::ControlType) -> Self::StateType;
}

// create an ExtendedKalmanFilter given a KalmanState S
pub struct ExtendedKalmanFilter<S>
where
    S: KalmanState,
    <<S as KalmanState>::StateLength as DimName>::Value: Mul<typenum::U1>,
    <<<S as KalmanState>::StateLength as DimName>::Value as Mul<typenum::U1>>::Output: ArrayLength<<S as KalmanState>::FloatType>,
    <<S as KalmanState>::StateLength as DimName>::Value: Mul,
    <<<S as KalmanState>::StateLength as DimName>::Value as Mul>::Output: ArrayLength<<S as KalmanState>::FloatType>
{
    state_pre: S,
    state_post: S,

    error_covariance_pre: Matrix<S::FloatType, S::StateLength, S::StateLength, ArrayStorage<S::FloatType, S::StateLength, S::StateLength>>,
    error_covariance_post: Matrix<S::FloatType, S::StateLength, S::StateLength, ArrayStorage<S::FloatType, S::StateLength, S::StateLength>>
}

impl<S> ExtendedKalmanFilter<S>
where
    S: KalmanState,
    <<S as KalmanState>::StateLength as DimName>::Value: Mul<typenum::U1>,
    <<<S as KalmanState>::StateLength as DimName>::Value as Mul<typenum::U1>>::Output: ArrayLength<<S as KalmanState>::FloatType>,
    <<S as KalmanState>::StateLength as DimName>::Value: Mul,
    <<<S as KalmanState>::StateLength as DimName>::Value as Mul>::Output: ArrayLength<<S as KalmanState>::FloatType>
{
    fn predict<F, U>(&mut self, system_model: F, control: <F as SystemModel>::ControlType)
    where
        F: SystemModel<StateType = S>,
        <<<F as SystemModel>::StateType as KalmanState>::StateLength as DimName>::Value: Mul<typenum::U1>,
        <<<<F as SystemModel>::StateType as KalmanState>::StateLength as DimName>::Value as Mul<typenum::U1>>::Output: ArrayLength<<<F as SystemModel>::StateType as KalmanState>::FloatType>,
        <<<F as SystemModel>::ControlType as ControlInput>::ControlLength as DimName>::Value: Mul<typenum::U1>,
        <<<<F as SystemModel>::ControlType as ControlInput>::ControlLength as DimName>::Value as Mul<typenum::U1>>::Output: ArrayLength<<<F as SystemModel>::ControlType as ControlInput>::FloatType>,
    {
        self.state_pre = system_model.f(&self.state_post, control);
    }
    fn update(&self/*, observation model, measurement*/)
    {

    }
}

//pub trait LinearizedSystemModel<N, DP, CP>: SystemModel<N, DP, CP>
//where
//    N: Real,
//    DP: DimName,
//    CP: DimName,
//    <DP as DimName>::Value: Mul<typenum::U1>,
//    <<DP as DimName>::Value as Mul<typenum::U1>>::Output: ArrayLength<N>,
//    <DP as DimName>::Value: Mul,
//    <<DP as DimName>::Value as Mul>::Output: ArrayLength<N>,
//    <CP as DimName>::Value: Mul<typenum::U1>,
//    <<CP as DimName>::Value as Mul<typenum::U1>>::Output: ArrayLength<N>,
//{
//    fn update_jacobians(&self, control: Vector<N, CP, ArrayStorage<N, CP, U1>>);
//}

pub struct ConstantVelocity1DState
{
    x: Vector<f32, na::dimension::U2, ArrayStorage<f32, na::dimension::U2, U1>>
}

impl ConstantVelocity1DState
{
    pub fn pos(&self) -> &f32
    {
        &self.x[(0,0)]
    }

    pub fn vel(&self) -> &f32
    {
        &self.x[(1,0)]
    }
}

impl KalmanState for ConstantVelocity1DState
{
    // useful to tell the length of the state vector when we construct a filter?
    type StateLength = na::dimension::U2;
    type FloatType = f32;

    fn x(&self) -> &Vector<Self::FloatType, Self::StateLength, ArrayStorage<Self::FloatType, Self::StateLength, U1>>
    { &self.x }
}

//// constant velocity model
//impl SystemModel<f32, na::dimension::U2, na::dimension::U0> for ConstantVelocity1DState
//{
//    fn f(&self, _control: Matrix<f32, na::dimension::U0, U1, ArrayStorage<f32, na::dimension::U0, U1>>)
//         -> Vector<f32, na::dimension::U2, ArrayStorage<f32, na::dimension::U2, U1>> {
//        let pos = *self.pos() + *self.vel();
//        let vel = *self.vel();
//
//        na::Matrix2x1::new(
//            pos,
//            vel
//        )
//    }
//}