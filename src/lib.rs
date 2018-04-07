#![feature(const_fn)]
#![feature(proc_macro)]
#![feature(unsize)]
#![no_std]

extern crate mat;
extern crate typenum;
extern crate generic_array;

use core::ops::{Deref, DerefMut, Mul, Add};

use mat::{Mat, MatrixArray};
use mat::mat;
use mat::traits::{Matrix, Zero};
use mat::dimension::DimName;
use typenum::{Unsigned, U0, U1, U2, Prod};
use generic_array::{GenericArray, ArrayLength, arr, arr_impl};


#[derive(Clone)]
pub struct KalmanFilterMat<T, DP>
    where
        T: Copy + Zero + Default,
        DP: Unsigned,
//    MP: Unsigned,
//    CP: Unsigned
{
    state_pre: Mat<T, [T; 1], DP, U1>
}

impl<T, DP> KalmanFilterMat<T, DP>
    where
        T: Copy + Zero + Default,
        DP: Unsigned,
{
//    fn init() -> KalmanFilterMat<T, DP> {
//            // how to initialize
//    }
}

pub struct KalmanFilterMatGen<T, DP>
where
    T: Copy + Zero + Default,
    DP: DimName,
    DP::Value: Mul<DP::Value>,
    Prod<DP::Value, DP::Value>: ArrayLength<T>,
{
    state_pre: MatrixArray<T, DP, DP>
}

impl<T, DP> KalmanFilterMatGen<T, DP>
    where
        T: Copy + Zero + Default,
        DP: DimName,
        DP::Value: Mul<DP::Value>,
        Prod<DP::Value, DP::Value>: ArrayLength<T>,
{
    fn init() -> Self {
        KalmanFilterMatGen {
            state_pre: Default::default()
        }
    }
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

    let a: Mat<i32, [i32; 2], U2, U1> = unsafe {
        Mat::new([0; 2])
    };

    let a = arr![u32; 1, 2, 3];

    let b : GenericArray<u32, U2> = Default::default();

    type G = typenum::Sum<U0, U1>;
    G::to_u32();

    // partially evaluate the tree
    assert_eq!(c.get(0, 0), 22);
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {

        assert_eq!(2 + 2, 4);
    }
}