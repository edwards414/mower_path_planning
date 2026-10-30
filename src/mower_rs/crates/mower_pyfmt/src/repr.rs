//! `repr(float)`: CPython's `float_repr`, which is
//! `PyOS_double_to_string(x, 'r', 0, Py_DTSF_ADD_DOT_0, NULL)`
//! (Objects/floatobject.c, Python/pystrtod.c `format_float_short`).

/// `repr(x)` for a Python float.
///
/// - The digits are the shortest that round-trip, the nearest such string,
///   ties to even (mode 0 of David Gay's dtoa); see `shortest`.
/// - With the decimal point `decpt` digits into them, the exponent form is
///   used when `decpt <= -4` or `decpt > 16`, that is for values under 1e-4
///   or from 1e16 on: `1e-05`, `1.5e+16` (sign always, at least two exponent
///   digits, no `.0` after a single digit).
/// - Otherwise fixed notation, with `.0` added to an integral value:
///   `100.0`, `0.0001`.
/// - `nan`, `inf`, `-inf`, `-0.0`.
pub fn float(x: f64) -> String {
    if x.is_nan() {
        return "nan".to_string();
    }
    if x.is_infinite() {
        return if x > 0.0 { "inf" } else { "-inf" }.to_string();
    }
    if x == 0.0 {
        return if x.is_sign_negative() { "-0.0" } else { "0.0" }.to_string();
    }
    let neg = x.is_sign_negative();
    let sci = shortest(x.abs());
    let (mantissa, exp) = sci.split_once('e').expect("{:e} has an exponent");
    let exp: i32 = exp.parse().expect("{:e} exponent");
    let digits: String = mantissa.chars().filter(|c| *c != '.').collect();
    let decpt = exp + 1;
    let mut out = String::with_capacity(digits.len() + 8);
    if neg {
        out.push('-');
    }
    if decpt <= -4 || decpt > 16 {
        out.push_str(&digits[..1]);
        if digits.len() > 1 {
            out.push('.');
            out.push_str(&digits[1..]);
        }
        out.push('e');
        out.push(if exp < 0 { '-' } else { '+' });
        let a = exp.unsigned_abs();
        if a < 10 {
            out.push('0');
        }
        out.push_str(&a.to_string());
    } else if decpt <= 0 {
        out.push_str("0.");
        for _ in 0..(-decpt) {
            out.push('0');
        }
        out.push_str(&digits);
    } else {
        let d = decpt as usize;
        if digits.len() <= d {
            out.push_str(&digits);
            for _ in digits.len()..d {
                out.push('0');
            }
            out.push_str(".0");
        } else {
            out.push_str(&digits[..d]);
            out.push('.');
            out.push_str(&digits[d..]);
        }
    }
    out
}

/// The shortest round-trip digits of a positive finite `x`, as `d.ddde<exp>`,
/// chosen as dtoa mode 0 chooses them. Rust's `{:e}` finds how many digits
/// are needed, but when two strings of that length round-trip it may not
/// take the one nearest `x`, and on an exact tie it rounds up where dtoa
/// rounds half to even (`911658185177222.25` is `...222.2` in Python, not
/// `...222.3`). Rounding `x` itself to that many digits (`{:.N$e}`, exact and
/// half-even) gives the nearest such string; it is the answer whenever it
/// still round-trips. When it does not (at a power of two, where the gap
/// below is half the gap above), the shortest one is the only candidate.
fn shortest(x: f64) -> String {
    let sci = format!("{:e}", x);
    let n = sci.split_once('e').map_or(0, |(m, _)| m.chars().filter(|c| c.is_ascii_digit()).count());
    let exact = format!("{:.*e}", n.saturating_sub(1), x);
    if exact != sci && exact.parse::<f64>() == Ok(x) {
        exact
    } else {
        sci
    }
}

#[cfg(test)]
mod tests {
    use super::float;

    #[test]
    #[allow(clippy::excessive_precision)] // 911658185177222.25 is exact: the tie case
    fn edges() {
        let cases: &[(f64, &str)] = &[
            (0.0, "0.0"),
            (-0.0, "-0.0"),
            (1.0, "1.0"),
            (-1.5, "-1.5"),
            (0.1, "0.1"),
            (0.0001, "0.0001"),
            (0.00001, "1e-05"),
            (1.5e-5, "1.5e-05"),
            (1e15, "1000000000000000.0"),
            (1e16, "1e+16"),
            (1.5e16, "1.5e+16"),
            (1e17, "1e+17"),
            (123456789.0, "123456789.0"),
            (5e-324, "5e-324"),
            (f64::MAX, "1.7976931348623157e+308"),
            (f64::from_bits(1), "5e-324"),
            (12.3, "12.3"),
            (1e100, "1e+100"),
            (9999999999999998.0, "9999999999999998.0"),
            // a tie between two 16-digit strings: dtoa rounds half to even
            (911658185177222.25, "911658185177222.2"),
            (-911658185177222.25, "-911658185177222.2"),
            (f64::NAN, "nan"),
            (f64::INFINITY, "inf"),
            (f64::NEG_INFINITY, "-inf"),
        ];
        for (x, want) in cases {
            assert_eq!(float(*x), *want, "repr({x:e})");
        }
    }
}
