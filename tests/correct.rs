extern crate rudie;
extern crate assert as asrt;

#[test]
fn single_correct_f64() {
    use rudie::KalmanFilter;
    use rudie::na::{U0, U1, U2, Vector, MatrixArray};
    use asrt::close;

    let epsilon = 0.0000001;

    let mut kf: KalmanFilter<f64, U2, U1, U0> = KalmanFilter::init();

    kf.measurement_matrix = rudie::na::Matrix1x2::new(
        1., 2.
    );

    kf.state_pre = rudie::na::Matrix2x1::new(
        1.,
        2.
    );

    kf.error_cov_pre = rudie::na::Matrix2::new(
        1., 2.,
        2., 1.
    );

    kf.measurement_noise_cov = rudie::na::Vector1::new(0.5);

    let measurement: Vector<f64, U1, MatrixArray<f64, U1, U1>> = rudie::na::Vector1::new(2.0);

    kf.correct(measurement);

    close(kf.residual[(0,0)], -3., epsilon);

    close(kf.innov_cov[(0,0)], 13.5, epsilon);

    close(kf.gain[(0,0)], 5./13.5, epsilon);
    close(kf.gain[(1,0)], 4./13.5, epsilon);

    close(kf.state_post[(0,0)], -1.5/13.5, epsilon);
    close(kf.state_post[(1,0)], 15./13.5, epsilon);

    close(kf.error_cov_post[(0,0)], -11.5/13.5, epsilon);
    close(kf.error_cov_post[(0,1)], 7./13.5, epsilon);
    close(kf.error_cov_post[(1,0)], 7./13.5, epsilon);
    close(kf.error_cov_post[(1,1)], -2.5/13.5, epsilon);
}

#[test]
fn single_correct_f32() {
    use rudie::KalmanFilter;
    use rudie::na::{U0, U1, U2, Vector, MatrixArray};
    use asrt::close;

    let epsilon = 0.0000001;

    let mut kf: KalmanFilter<f32, U2, U1, U0> = KalmanFilter::init();

    kf.measurement_matrix = rudie::na::Matrix1x2::new(
        1., 2.
    );

    kf.state_pre = rudie::na::Matrix2x1::new(
        1.,
        2.
    );

    kf.error_cov_pre = rudie::na::Matrix2::new(
        1., 2.,
        2., 1.
    );

    kf.measurement_noise_cov = rudie::na::Vector1::new(0.5);

    let measurement: Vector<f32, U1, MatrixArray<f32, U1, U1>> = rudie::na::Vector1::new(2.0);

    kf.correct(measurement);

    close(kf.residual[(0,0)], -3., epsilon);

    close(kf.innov_cov[(0,0)], 13.5, epsilon);

    close(kf.gain[(0,0)], 5./13.5, epsilon);
    close(kf.gain[(1,0)], 4./13.5, epsilon);

    close(kf.state_post[(0,0)], -1.5/13.5, epsilon);
    close(kf.state_post[(1,0)], 15./13.5, epsilon);

    close(kf.error_cov_post[(0,0)], -11.5/13.5, epsilon);
    close(kf.error_cov_post[(0,1)], 7./13.5, epsilon);
    close(kf.error_cov_post[(1,0)], 7./13.5, epsilon);
    close(kf.error_cov_post[(1,1)], -2.5/13.5, epsilon);
}