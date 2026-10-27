//! A seeded generator for the sensor noise (SplitMix64 and Box-Muller), so
//! a run's noise is the same every time.

pub struct Rng {
    state: u64,
    spare: Option<f64>,
}

impl Rng {
    pub fn new(seed: u64) -> Rng {
        Rng { state: seed, spare: None }
    }

    pub fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// (0, 1]
    fn unit(&mut self) -> f64 {
        ((self.next_u64() >> 11) + 1) as f64 / (1u64 << 53) as f64
    }

    /// A standard normal sample.
    pub fn normal(&mut self) -> f64 {
        if let Some(z) = self.spare.take() {
            return z;
        }
        let (u, v) = (self.unit(), self.unit());
        let r = (-2.0 * u.ln()).sqrt();
        let a = std::f64::consts::TAU * v;
        self.spare = Some(r * a.sin());
        r * a.cos()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn seeded_and_standard_normal() {
        let mut a = Rng::new(7);
        let mut b = Rng::new(7);
        let xs: Vec<f64> = (0..20_000).map(|_| a.normal()).collect();
        assert!(xs.iter().zip((0..20_000).map(|_| b.normal())).all(|(x, y)| *x == y), "the same seed, the same noise");
        let mean = xs.iter().sum::<f64>() / xs.len() as f64;
        let var = xs.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / xs.len() as f64;
        assert!(mean.abs() < 0.03 && (var - 1.0).abs() < 0.05, "{mean} {var}");
        assert_ne!(Rng::new(1).next_u64(), Rng::new(2).next_u64());
    }
}
