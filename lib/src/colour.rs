use crate::scan::Picometers;

const FULL: u32 = 255;
const DIM: u32 = 77;

/// Approximate RGB (0..=255 per channel) of light at `wavelength`, after Dan
/// Bruton's piecewise model. Outside 380..=780 nm the result is black.
pub fn wavelength_rgb(wavelength: Picometers) -> [u8; 3] {
    let [red, green, blue] = hue(wavelength);
    let intensity = intensity(wavelength);
    [red, green, blue].map(|channel| (channel * intensity / FULL) as u8)
}

fn hue(nm: Picometers) -> [u32; 3] {
    match nm {
        380_000..440_000 => [FULL - ramp(nm, 380_000, 440_000), 0, FULL],
        440_000..490_000 => [0, ramp(nm, 440_000, 490_000), FULL],
        490_000..510_000 => [0, FULL, FULL - ramp(nm, 490_000, 510_000)],
        510_000..580_000 => [ramp(nm, 510_000, 580_000), FULL, 0],
        580_000..645_000 => [FULL, FULL - ramp(nm, 580_000, 645_000), 0],
        645_000..=780_000 => [FULL, 0, 0],
        _ => [0, 0, 0],
    }
}

fn intensity(nm: Picometers) -> u32 {
    match nm {
        380_000..420_000 => DIM + ramp(nm, 380_000, 420_000) * (FULL - DIM) / FULL,
        420_000..700_000 => FULL,
        700_000..=780_000 => DIM + (FULL - ramp(nm, 700_000, 780_000)) * (FULL - DIM) / FULL,
        _ => 0,
    }
}

fn ramp(nm: Picometers, from: Picometers, to: Picometers) -> u32 {
    let progress = i64::from(nm - from) * i64::from(FULL) / i64::from(to - from);
    progress.clamp(0, i64::from(FULL)) as u32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wavelength_rgb_cases() {
        let cases: &[(&str, Picometers, [u8; 3])] = &[
            ("ultraviolet", 300_000, [0, 0, 0]),
            ("violet edge", 380_000, [77, 0, 77]),
            ("blue", 440_000, [0, 0, 255]),
            ("cyan", 490_000, [0, 255, 255]),
            ("green", 510_000, [0, 255, 0]),
            ("yellow", 580_000, [255, 255, 0]),
            ("red", 645_000, [255, 0, 0]),
            ("deep red", 700_000, [255, 0, 0]),
            ("red edge", 780_000, [77, 0, 0]),
            ("infrared", 800_000, [0, 0, 0]),
            ("half way to green", 545_000, [127, 255, 0]),
        ];
        for (name, wavelength, expected) in cases {
            assert_eq!(&wavelength_rgb(*wavelength), expected, "{name}");
        }
    }
}
