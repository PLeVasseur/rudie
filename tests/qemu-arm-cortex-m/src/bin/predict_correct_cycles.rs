#![no_main]
#![no_std]

//extern crate panic_halt;
extern crate panic_semihosting;

use cortex_m_rt::entry;
use cortex_m_semihosting::{debug};

#[macro_use]
extern crate assert_float_eq;

#[entry]
fn main() -> ! {

    extern crate rudie;
    extern crate libm;

    use assert_float_eq::{assert_float_absolute_eq};

    use rudie::OpenCVKalmanFilter;
    use rudie::na::{U0, U1, U2, Vector, ArrayStorage};

    let state_buffer = [0.040157366134307, -0.004584667101862575, -0.13210335109501573,
        -0.06167960574363984, -0.09218946743133555, 0.04102534430132926, 0.055414343550211326,
        0.023937347653503412, 0.004426223901518564, -0.012629592056939521];
    let /*mut*/ state_buffer_ndx = 0;

    let measurement_noise_buffer = [0.040157366134307, -0.004584667101862575,
        -0.13210335109501573, -0.06167960574363984, -0.09218946743133555, 0.04102534430132926,
        0.055414343550211326, 0.023937347653503412, 0.004426223901518564, -0.012629592056939521];
    let mut measurement_noise_buffer_ndx = 0;

    let process_noise_buffer = [0.0012698874181772126, -0.0001449799035552893, -0.004177474760011482,
        -0.0019504803933110553, -0.002915286933609328, 0.0012973352978480962, 0.00175235540661729,
        0.0007569653972835781, 0.00013996948962675476, -0.00039938276818699854];
    let mut process_noise_buffer_ndx = 0;

    /**************************************************
    ** filter configurations to assert against - begin
    ***************************************************/

    // known filter configurations to assert state against for kalman predict
    let state_pre_assert_predict = rudie::na::Matrix2x3::new(
        -0.19378295683865557, 0.1361050053334287,   0.04097848073026808,
        -0.06167960574363984, 0.06884248182130964, -0.00403224753640588
    );
    let error_cov_pre_assert_predict = rudie::na::Matrix2x6::new(
        2.00001, 1.,      0.7143075510116619, 0.57144061223518,   0.3509028377933237, 0.1929981748186918,
        1.,      1.00001, 0.57144061223518,   0.5238317913724221, 0.1929981748186918, 0.1228331394128128
    );

    // known filter configurations to assert state against for kalman correct
    let residual_assert_correct = rudie::na::Matrix1x3::new(
        0.27409768910726956, -0.10384708598466963, -0.1451463671400915
    );
    let innov_cov_assert_correct = rudie::na::Matrix1x3::new(
        2.10001, 0.8143075510116619, 0.45090283779332374
    );
    let gain_assert_correct = rudie::na::Matrix2x3::new(
        0.9523811791372422, 0.8771962756875286, 0.7782227308894513,
        0.47618820862757794, 0.70175035405879,  0.4280260815460971
    );
    let state_post_assert_correct = rudie::na::Matrix2x3::new(
        0.06726252351211906, 0.04501072826667396,  -0.07197772148417683,
        0.06884248182130964, -0.00403224753640588, -0.06615867831403044
    );
    let error_cov_post_assert_correct = rudie::na::Matrix2x6::new(
        0.09523811791372422, 0.04761882086275779, 0.08771962756875286, 0.07017503540587901, 0.07782227308894514, 0.04280260815460972,
        0.04761882086275779, 0.5238217913724221,  0.07017503540587901, 0.12282313941281281, 0.04280260815460972, 0.04022488689961951
    );

    /**************************************************
    ** filter configurations to assert against - end
    ***************************************************/

    // We use the typenum crate here to specify type-level numbers such as U0 = 0, U1 = 1, ...
    // until Rust has support for const generics
    let mut kf: OpenCVKalmanFilter<f64, U2, U1, U0> = OpenCVKalmanFilter::init();

    // (phi, delta_phi), i.e. orientation and angular rate
    let mut state: Vector<f64, U2, ArrayStorage<f64, U2, U1>>;

    let mut process_noise: Vector<f64, U2, ArrayStorage<f64, U2, U1>>;

    let mut measurement: Vector<f64, U1, ArrayStorage<f64, U1, U1>>;

    state = rudie::na::Matrix2x1::new(
        state_buffer[state_buffer_ndx],
        state_buffer[state_buffer_ndx+1]
    );
//    state_buffer_ndx += 2;
    kf.transition_matrix = rudie::na::Matrix2::new(
        1., 1.,
        0., 1.
    );

    kf.measurement_matrix = rudie::na::Matrix1x2::identity();
    kf.process_noise_cov = rudie::na::Matrix2::from_diagonal_element(1e-5);
    kf.measurement_noise_cov = rudie::na::Matrix1::from_diagonal_element(1e-1);
    kf.error_cov_post = rudie::na::Matrix2::from_diagonal_element(1.);

    // set up the initial state
    kf.state_post = rudie::na::Matrix2x1::new(
        -0.13210335109501573,
        -0.06167960574363984
    );

    for cycle in 0..3 {
        // Kalman predict
        kf.predict_no_control();

        /********************************************************************
        ** assert known filter configuration after predict_no_control - begin
        *********************************************************************/
        assert_float_absolute_eq!(kf.state_pre[(0,0)], state_pre_assert_predict[(0,cycle)], core::f64::EPSILON);
        assert_float_absolute_eq!(kf.state_pre[(1,0)], state_pre_assert_predict[(1,cycle)], core::f64::EPSILON);

        assert_float_absolute_eq!(kf.error_cov_pre[(0,0)], error_cov_pre_assert_predict[(0,cycle*2)], core::f64::EPSILON);
        assert_float_absolute_eq!(kf.error_cov_pre[(1,0)], error_cov_pre_assert_predict[(1,cycle*2)], core::f64::EPSILON);
        assert_float_absolute_eq!(kf.error_cov_pre[(0,1)], error_cov_pre_assert_predict[(0,cycle*2+1)], core::f64::EPSILON);
        assert_float_absolute_eq!(kf.error_cov_pre[(1,1)], error_cov_pre_assert_predict[(1,cycle*2+1)], core::f64::EPSILON);
        /********************************************************************
        ** assert known filter configuration after predict_no_control - end
        *********************************************************************/

        // generate measurement
        measurement = rudie::na::Matrix1::new(
            measurement_noise_buffer[measurement_noise_buffer_ndx]
        );
        measurement_noise_buffer_ndx += 1;
        measurement += kf.measurement_matrix*state;

        // Kalman correct
        kf.correct(measurement);

        /*************************************************************
        ** assert known filter configuration after correct - begin
        **************************************************************/
        assert_float_absolute_eq!(kf.residual[(0,0)], residual_assert_correct[(0, cycle)], core::f64::EPSILON);

        assert_float_absolute_eq!(kf.innov_cov[(0, 0)], innov_cov_assert_correct[(0, cycle)], core::f64::EPSILON);

        assert_float_absolute_eq!(kf.gain[(0, 0)], gain_assert_correct[(0, cycle)], core::f64::EPSILON);
        assert_float_absolute_eq!(kf.gain[(1, 0)], gain_assert_correct[(1, cycle)], core::f64::EPSILON);

        assert_float_absolute_eq!(kf.state_post[(0,0)], state_post_assert_correct[(0, cycle)], core::f64::EPSILON);
        assert_float_absolute_eq!(kf.state_post[(1,0)], state_post_assert_correct[(1, cycle)], core::f64::EPSILON);

        assert_float_absolute_eq!(kf.error_cov_post[(0,0)], error_cov_post_assert_correct[(0, cycle*2)], core::f64::EPSILON);
        assert_float_absolute_eq!(kf.error_cov_post[(1,0)], error_cov_post_assert_correct[(1, cycle*2)], core::f64::EPSILON);
        assert_float_absolute_eq!(kf.error_cov_post[(0,1)], error_cov_post_assert_correct[(0, cycle*2+1)], core::f64::EPSILON);
        assert_float_absolute_eq!(kf.error_cov_post[(1,1)], error_cov_post_assert_correct[(1, cycle*2+1)], core::f64::EPSILON);
        /*************************************************************
        ** assert known filter configuration after correct - end
        **************************************************************/

        // generate next state
        process_noise = rudie::na::Matrix2x1::new(
            process_noise_buffer[process_noise_buffer_ndx],
            process_noise_buffer[process_noise_buffer_ndx+1]
        );
        process_noise_buffer_ndx += 2;
        state = kf.transition_matrix * state + process_noise;
    }

    // exit QEMU or the debugger section
    debug::exit(debug::EXIT_SUCCESS);

    loop {}
}
