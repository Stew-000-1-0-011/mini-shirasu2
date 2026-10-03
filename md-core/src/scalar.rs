use core::ops::{Add, Sub, Mul, Div, Neg, AddAssign, SubAssign};

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct Scalar {
	v: f32,
}

impl Scalar {
	pub const fn zero() -> Self {
		Scalar { v: 0.0 }
	}

	pub const fn one() -> Self {
		Scalar { v: 1.0 }
	}

	pub const fn min(self, rhs: Self) -> Self {
		Scalar { v: self.v.min(rhs.v) }
	}

	pub const fn max(self, rhs: Self) -> Self {
		Scalar { v: self.v.max(rhs.v) }
	}

	pub const fn recip(self) -> Self {
		Scalar { v: self.v.recip() }
	}

	pub const fn abs(self) -> Self {
		Scalar { v: self.v.abs() }
	}

	pub fn sat(self, threshold_inv: Self) -> Self {
		(self * threshold_inv).min(Self::one()).max(-Self::one())
	}
}

impl From<f32> for Scalar {
	fn from(v: f32) -> Self {
		Scalar { v }
	}
}

impl From<Scalar> for f32 {
	fn from(s: Scalar) -> Self {
		s.v
	}
}

impl Add for Scalar {
	type Output = Self;
	fn add(self, rhs: Self) -> Self::Output {
		Scalar { v: self.v + rhs.v }
	}
}

impl Sub for Scalar {
	type Output = Self;
	fn sub(self, rhs: Self) -> Self::Output {
		Scalar { v: self.v - rhs.v }
	}
}

impl Mul for Scalar {
	type Output = Self;
	fn mul(self, rhs: Self) -> Self::Output {
		Scalar { v: self.v * rhs.v }
	}
}

impl Div for Scalar {
	type Output = Self;
	fn div(self, rhs: Self) -> Self::Output {
		Scalar { v: self.v / rhs.v }
	}
}

impl Neg for Scalar {
	type Output = Self;
	fn neg(self) -> Self::Output {
		Scalar { v: -self.v }
	}
}

impl AddAssign for Scalar {
	fn add_assign(&mut self, rhs: Self) {
		self.v += rhs.v;
	}
}

impl SubAssign for Scalar {
	fn sub_assign(&mut self, rhs: Self) {
		self.v -= rhs.v;
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn zero_is_0() {
		assert_eq!(Scalar::zero(), Scalar::from(0.0));
	}

	#[test]
	fn one_is_1() {
		assert_eq!(Scalar::one(), Scalar::from(1.0));
	}

	#[test]
	fn add() {
		assert_eq!(Scalar::from(1.5) + Scalar::from(2.5), Scalar::from(4.0));
	}

	#[test]
	fn sub() {
		assert_eq!(Scalar::from(5.0) - Scalar::from(2.0), Scalar::from(3.0));
	}

	#[test]
	fn mul() {
		assert_eq!(Scalar::from(3.0) * Scalar::from(4.0), Scalar::from(12.0));
	}

	#[test]
	fn div() {
		assert_eq!(Scalar::from(6.0) / Scalar::from(2.0), Scalar::from(3.0));
		assert_eq!(Scalar::from(1.0) / Scalar::from(-4.0), Scalar::from(-0.25));
	}

	#[test]
	fn recip_is_reciprocal() {
		assert_eq!(Scalar::from(4.0).recip(), Scalar::from(0.25));
		assert_eq!(Scalar::from(-2.0).recip(), Scalar::from(-0.5));
	}

	#[test]
	fn abs_drops_sign() {
		assert_eq!(Scalar::from(-3.0).abs(), Scalar::from(3.0));
		assert_eq!(Scalar::from(3.0).abs(), Scalar::from(3.0));
	}

	#[test]
	fn neg() {
		assert_eq!(-Scalar::from(3.0), Scalar::from(-3.0));
	}

	#[test]
	fn add_assign() {
		let mut a = Scalar::from(1.0);
		a += Scalar::from(2.0);
		assert_eq!(a, Scalar::from(3.0));
	}

	#[test]
	fn sub_assign() {
		let mut a = Scalar::from(5.0);
		a -= Scalar::from(2.0);
		assert_eq!(a, Scalar::from(3.0));
	}

	#[test]
	fn min_picks_smaller() {
		assert_eq!(Scalar::from(1.0).min(Scalar::from(2.0)), Scalar::from(1.0));
		assert_eq!(Scalar::from(2.0).min(Scalar::from(1.0)), Scalar::from(1.0));
	}

	#[test]
	fn max_picks_larger() {
		assert_eq!(Scalar::from(1.0).max(Scalar::from(2.0)), Scalar::from(2.0));
		assert_eq!(Scalar::from(2.0).max(Scalar::from(1.0)), Scalar::from(2.0));
	}

	#[test]
	fn sat_passes_through_within_threshold() {
		// threshold = 2.0 => threshold_inv = 0.5, so |self| <= 2.0 は素通り
		let threshold_inv = Scalar::from(0.5);
		assert_eq!(Scalar::from(1.0).sat(threshold_inv), Scalar::from(0.5));
	}

	#[test]
	fn sat_clamps_above_threshold() {
		let threshold_inv = Scalar::from(0.5);
		assert_eq!(Scalar::from(10.0).sat(threshold_inv), Scalar::from(1.0));
	}

	#[test]
	fn sat_clamps_below_negative_threshold() {
		let threshold_inv = Scalar::from(0.5);
		assert_eq!(Scalar::from(-10.0).sat(threshold_inv), Scalar::from(-1.0));
	}
}
