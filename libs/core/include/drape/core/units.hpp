#pragma once

// Conversions from the units shown in the UI to the SI units used internally.
namespace drape::units {

constexpr double mmToM(double mm) { return mm * 0.001; }
constexpr double cmToM(double cm) { return cm * 0.01; }
constexpr double inToM(double in) { return in * 0.0254; }
constexpr double gsmToKgPerM2(double gsm) { return gsm * 0.001; }
constexpr double microNewtonMeterToNewtonMeter(double uNm) { return uNm * 1e-6; }
constexpr double percentToFraction(double percent) { return percent * 0.01; }
constexpr double ozPerSqYdToGsm(double oz) { return oz * 33.9057; }

}  // namespace drape::units
