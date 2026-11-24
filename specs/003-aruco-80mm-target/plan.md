# Implementation Plan: 80 mm ArUco Landing Target

1. Parameterize the existing A4 target generator by physical marker size.
2. Generate `DICT_5X5_50`, ID 0 at 80 mm from the already verified ID 0
   pattern.
3. Synchronize detector defaults, runtime YAML and operator documentation.
4. Add a configuration regression test and validate PDF/PNG dimensions.
5. Leave camera calibration and control gains unchanged until hardware data
   shows that retuning is necessary.
