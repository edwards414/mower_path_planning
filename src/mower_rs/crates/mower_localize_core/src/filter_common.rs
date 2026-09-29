//! `robot_localization/filter_common.hpp`: the state layout shared by the
//! filter core and the measurement preprocessing.

pub const STATE_SIZE: usize = 15;

pub const STATE_X: usize = 0;
pub const STATE_Y: usize = 1;
pub const STATE_Z: usize = 2;
pub const STATE_ROLL: usize = 3;
pub const STATE_PITCH: usize = 4;
pub const STATE_YAW: usize = 5;
pub const STATE_VX: usize = 6;
pub const STATE_VY: usize = 7;
pub const STATE_VZ: usize = 8;
pub const STATE_VROLL: usize = 9;
pub const STATE_VPITCH: usize = 10;
pub const STATE_VYAW: usize = 11;
pub const STATE_AX: usize = 12;
pub const STATE_AY: usize = 13;
pub const STATE_AZ: usize = 14;

pub const CONTROL_VX: usize = 0;
pub const CONTROL_VY: usize = 1;
pub const CONTROL_VZ: usize = 2;
pub const CONTROL_VROLL: usize = 3;
pub const CONTROL_VPITCH: usize = 4;
pub const CONTROL_VYAW: usize = 5;

pub const POSITION_OFFSET: usize = STATE_X;
pub const ORIENTATION_OFFSET: usize = STATE_ROLL;
pub const POSITION_V_OFFSET: usize = STATE_VX;
pub const ORIENTATION_V_OFFSET: usize = STATE_VROLL;
pub const POSITION_A_OFFSET: usize = STATE_AX;

pub const POSE_SIZE: usize = 6;
pub const TWIST_SIZE: usize = 6;
pub const POSITION_SIZE: usize = 3;
pub const ORIENTATION_SIZE: usize = 3;
pub const ACCELERATION_SIZE: usize = 3;

/// A 15x15 dense matrix, row major, as the filter uses everywhere.
pub type Mat15 = [[f64; STATE_SIZE]; STATE_SIZE];
/// A 15-element state/measurement vector.
pub type Vec15 = [f64; STATE_SIZE];

pub const ZERO_MAT15: Mat15 = [[0.0; STATE_SIZE]; STATE_SIZE];

pub fn identity_mat15() -> Mat15 {
    let mut m = ZERO_MAT15;
    for (i, row) in m.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    m
}

/// `C = A * B` for 15x15 matrices.
pub fn matmul15(a: &Mat15, b: &Mat15) -> Mat15 {
    let mut c = ZERO_MAT15;
    for i in 0..STATE_SIZE {
        for k in 0..STATE_SIZE {
            let aik = a[i][k];
            if aik == 0.0 {
                continue;
            }
            for j in 0..STATE_SIZE {
                c[i][j] += aik * b[k][j];
            }
        }
    }
    c
}

/// `C = A * B^T` for 15x15 matrices.
pub fn matmul15_transpose(a: &Mat15, b: &Mat15) -> Mat15 {
    let mut c = ZERO_MAT15;
    for i in 0..STATE_SIZE {
        for j in 0..STATE_SIZE {
            let mut s = 0.0;
            for k in 0..STATE_SIZE {
                s += a[i][k] * b[j][k];
            }
            c[i][j] = s;
        }
    }
    c
}

/// `y = A * x` for a 15x15 matrix and a 15-vector.
pub fn matvec15(a: &Mat15, x: &Vec15) -> Vec15 {
    let mut y = [0.0f64; STATE_SIZE];
    for i in 0..STATE_SIZE {
        let mut s = 0.0;
        for k in 0..STATE_SIZE {
            s += a[i][k] * x[k];
        }
        y[i] = s;
    }
    y
}

/// In-place inverse of an `n x n` matrix (n <= STATE_SIZE) by LU decomposition
/// with partial pivoting, matching what `Eigen::MatrixXd::inverse()` does for
/// dynamically sized matrices (it dispatches to `PartialPivLU`).
pub fn inverse_partial_piv_lu(a: &[[f64; STATE_SIZE]; STATE_SIZE], n: usize) -> Mat15 {
    let mut lu = *a;
    let mut perm: [usize; STATE_SIZE] = [0; STATE_SIZE];
    for (i, p) in perm.iter_mut().enumerate() {
        *p = i;
    }

    for k in 0..n {
        // Find the pivot row.
        let mut piv = k;
        let mut best = lu[k][k].abs();
        for (i, row) in lu.iter().enumerate().take(n).skip(k + 1) {
            let v = row[k].abs();
            if v > best {
                best = v;
                piv = i;
            }
        }
        if piv != k {
            lu.swap(piv, k);
            perm.swap(piv, k);
        }
        let pivot = lu[k][k];
        if pivot == 0.0 {
            continue;
        }
        for i in k + 1..n {
            lu[i][k] /= pivot;
            let lik = lu[i][k];
            for j in k + 1..n {
                lu[i][j] -= lik * lu[k][j];
            }
        }
    }

    // Solve LU * X = P for X, column by column.
    let mut inv = ZERO_MAT15;
    for col in 0..n {
        let mut x = [0.0f64; STATE_SIZE];
        for (i, xi) in x.iter_mut().enumerate().take(n) {
            *xi = if perm[i] == col { 1.0 } else { 0.0 };
        }
        // Forward substitution, unit lower triangular.
        for i in 1..n {
            let mut s = x[i];
            for j in 0..i {
                s -= lu[i][j] * x[j];
            }
            x[i] = s;
        }
        // Back substitution, upper triangular.
        for i in (0..n).rev() {
            let mut s = x[i];
            for j in i + 1..n {
                s -= lu[i][j] * x[j];
            }
            x[i] = s / lu[i][i];
        }
        for (i, xi) in x.iter().enumerate().take(n) {
            inv[i][col] = *xi;
        }
    }
    inv
}
