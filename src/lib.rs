#![feature(const_fn)]
#![feature(proc_macro)]
#![feature(unsize)]
#![no_std]

pub extern crate mat;
pub extern crate typenum;
pub extern crate generic_array;

use core::ops::{Mul};
use core::fmt;

use mat::{Mat, MatGen, MatGen2};
use mat::mat;
use mat::traits::{Matrix, Zero};
use typenum::{Unsigned, Prod, UInt, UTerm};
use typenum::consts::*;
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

pub struct KalmanFilterMatGen2<T, DP>
where
    T: Copy + Zero + Default,
    DP: Unsigned,
    DP: Mul<U1>,
    Prod<DP, U1>: ArrayLength<T>,
{
    state_pre: MatGen2<T, DP, U1>
}

impl<T, DP> KalmanFilterMatGen2<T, DP>
where
    T: Copy + Zero + Default,
    DP: Unsigned,
    DP: Mul<U1>,
    Prod<DP, U1>: ArrayLength<T>,
{
    pub fn init() -> Self {
        KalmanFilterMatGen2 {
            state_pre: Default::default()
        }
    }
}

impl<T, DP> fmt::Debug for KalmanFilterMatGen2<T, DP>
where
    T: Copy + Zero + Default + fmt::Debug,
    DP: Unsigned,
    DP: Mul<U1>,
    Prod<DP, U1>: ArrayLength<T>,
{
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.write_str("state_pre: ")?;
        write!(f, "{:?}", self.state_pre)?;
        f.write_str("]")
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

    type MYU1 = U1;

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