# Feature Specification: 80 mm ArUco Landing Target

**Feature Branch**: `main`
**Created**: 2026-11-24
**Status**: Implementation complete; print and camera validation pending

## Goal

Change the landing target from a 160 mm marker to an 80 mm outer black square
using `DICT_5X5_50`, ID 0, and provide a dimensionally accurate A4 PDF.

## Acceptance Criteria

- Runtime detector uses `DICT_5X5_50`, ID 0 and `marker_size_m: 0.08`.
- The A4 PDF and 300 DPI PNG contain an 80 x 80 mm outer black square.
- Calibration documentation identifies Actual size / 100% printing.
- Camera intrinsics, distortion and camera-to-body transforms remain unchanged.
- Software tests verify the runtime target contract.

## Safety Boundary

This change only updates perception geometry and printable assets. It does not
enable the landing PID, enable the PX4 bridge, arm, or request Offboard. The
smaller target must be validated on the bench before flight.
