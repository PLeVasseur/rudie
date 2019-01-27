extern crate rudie;

#[test]
fn single_predict_dummy_control_f64() {
    use rudie::OpenCVKalmanFilter;
    use rudie::na::{U0, U1, U2, Vector, MatrixArray, zero};

    let dummy_control: Vector<f64, U0, MatrixArray<f64, U0, U1>> = zero();
    // no control parameters, CP = U0
    let mut kf: OpenCVKalmanFilter<f64, U2, U1, U0> = OpenCVKalmanFilter::init();

    kf.state_post = rudie::na::Matrix2x1::new(
        1.,
        2.
    );

    kf.transition_matrix = rudie::na::Matrix2::new(
        2., 3.,
        4., 5.
    );

    kf.predict(dummy_control);

    assert_eq!(kf.state_pre[(0,0)], 8.);
    assert_eq!(kf.state_pre[(1,0)], 14.);
}

#[test]
fn single_predict_with_control_f64() {
    use rudie::OpenCVKalmanFilter;
    use rudie::na::{U1, U2, Vector, MatrixArray};

    let dummy_control: Vector<f64, U1, MatrixArray<f64, U1, U1>> = rudie::na::Vector1::new(1.0);

    let mut kf: OpenCVKalmanFilter<f64, U2, U1, U1> = OpenCVKalmanFilter::init();
    kf.control_matrix = rudie::na::Matrix2x1::new(
        1.,
        1.
    );

    kf.state_post = rudie::na::Matrix2x1::new(
        1.,
        2.
    );

    kf.transition_matrix = rudie::na::Matrix2::new(
        2., 3.,
        4., 5.
    );

    kf.predict(dummy_control);

    assert_eq!(kf.state_pre[(0,0)], 9.);
    assert_eq!(kf.state_pre[(1,0)], 15.);
}

#[test]
fn single_predict_dummy_control_f32() {
    use rudie::OpenCVKalmanFilter;
    use rudie::na::{U0, U1, U2, Vector, MatrixArray, zero};

    let dummy_control: Vector<f32, U0, MatrixArray<f32, U0, U1>> = zero();
    // no control parameters, CP = U0
    let mut kf: OpenCVKalmanFilter<f32, U2, U1, U0> = OpenCVKalmanFilter::init();

    kf.state_post = rudie::na::Matrix2x1::new(
        1.,
        2.
    );

    kf.transition_matrix = rudie::na::Matrix2::new(
        2., 3.,
        4., 5.
    );

    kf.predict(dummy_control);

    assert_eq!(kf.state_pre[(0,0)], 8.);
    assert_eq!(kf.state_pre[(1,0)], 14.);
}

#[test]
fn single_predict_with_control_f32() {
    use rudie::OpenCVKalmanFilter;
    use rudie::na::{U1, U2, Vector, MatrixArray};

    let dummy_control: Vector<f32, U1, MatrixArray<f32, U1, U1>> = rudie::na::Vector1::new(1.0);

    let mut kf: OpenCVKalmanFilter<f32, U2, U1, U1> = OpenCVKalmanFilter::init();
    kf.control_matrix = rudie::na::Matrix2x1::new(
        1.,
        1.
    );

    kf.state_post = rudie::na::Matrix2x1::new(
        1.,
        2.
    );

    kf.transition_matrix = rudie::na::Matrix2::new(
        2., 3.,
        4., 5.
    );

    kf.predict(dummy_control);

    assert_eq!(kf.state_pre[(0,0)], 9.);
    assert_eq!(kf.state_pre[(1,0)], 15.);
}