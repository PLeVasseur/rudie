#![feature(use_extern_macros)]

extern crate rudie;

#[test]
fn dummy_predict() {
    use rudie::KalmanFilter;
    use rudie::typenum::consts::*;
    use rudie::mat::mat_gen_imm;
    use rudie::mat::MatGenImm;
    use rudie::mat::traits::ImmMatrix;


    let dummy_control: MatGenImm<f32, U0, U1> = MatGenImm::default();
    // no control parameters, CP = U0
    let mut kf: KalmanFilter<f32, U2, U1, U0> = KalmanFilter::init();

    kf.state_post = mat_gen_imm![
        [1.],
        [2.]
    ];
    kf.transition_matrix = mat_gen_imm![
        [2., 3.],
        [4., 5.]
    ];

    kf.predict(dummy_control);

    assert_eq!(kf.state_pre.get(0,0), 8.);
    assert_eq!(kf.state_pre.get(1,0), 14.);
}
