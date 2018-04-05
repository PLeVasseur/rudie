#![feature(proc_macro)]

extern crate mat;
extern crate typenum;
extern crate core;

use mat::Mat;
use mat::mat;
use mat::traits::{Matrix, Zero};
use typenum::{UTerm, Unsigned, UInt, U1, U2};

//pub trait Zero<T> {
//    fn zero() -> T;
//}
//
//impl Zero<f32> for f32 {
//    fn zero() -> f32 {
//        0.0
//    }
//}

//struct KalmanFilterCV<T, DP, MP, CP>
#[derive(Clone)]
pub struct KalmanFilterCV<T, DP>
where
    T: Copy + Zero,
    DP: Unsigned,
//    MP: Unsigned,
//    CP: Unsigned
{
    state_pre: Mat<T, [T; 1], DP, U1>
}

impl<T, DP> KalmanFilterCV<T, DP>
where
    T: Copy + Zero,
    DP: Unsigned
{
//    fn init() -> KalmanFilterCV<T, DP> {
//        KalmanFilterCV {
//            state_pre: mat![
//                [T::zero()]
//            ]
////            state_pre: mat![
////                [T::zero()]
////            ]
//        }
//    }
}

fn build_matrix() {

    // 2 by 3 matrix
    let a = mat![
        [1, 2, 3],
        [3, 4, 5],
    ];

    // 3 by 2 matrix
    let b = mat![
        [1, 2],
        [3, 4],
        [5, 6],
    ];

    // build an expression tree
    let c = &a * &b;

    let d: [f32; 2] = [0.; 0.];

    let a: Mat<i32, [i32; 2], U2, U1> = unsafe {
        Mat::new([i32; 2])
    };

    // partially evaluate the tree
    assert_eq!(c.get(0, 0), 22);
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        f32::zero();



        assert_eq!(2 + 2, 4);
    }
}