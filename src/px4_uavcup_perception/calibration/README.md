# Printable calibration targets

Print every PDF with **Actual size / 100% scale**. Disable Fit to page and
Shrink oversized pages. PDF is preferred over PNG for dimensionally accurate
printing.

- `checkerboard_9x6_25mm_a4.pdf`: A4 landscape, 9x6 inner corners, 25 mm
  squares, 250x175 mm outer checker area. Use `square_size_m:=0.025`.
- `checkerboard_9x6_30mm_a3.svg`: A3 landscape, 9x6 inner corners, 30 mm
  squares. Use `square_size_m:=0.03`.
- `aruco_5x5_50/`: IDs 0 through 4 from `DICT_5X5_50`, one A4 portrait page
  per ID. Each marker's black outer square is 160x160 mm.

After printing, verify one checker square and both dimensions of an ArUco
marker with a ruler. The physical marker size used by pose estimation is the
outer black-square dimension, not the page or white margin.
