# GasTurbineEngineModel — Map File Format

## Overview

All engine performance maps use a common structured text format parsed
by `MapReader`. Map files are plain text, human readable, and designed
to be replaced with real engine data without modifying any C++ code.

## Supported Map Types

| TYPE       | Component               | Axes              | Outputs    |
|------------|-------------------------|-------------------|------------|
| COMPRESSOR | Fan, HP Compressor      | Wc [kg/s], Nc [%] | PR, eff    |
| TURBINE    | HP Turbine, LP Turbine  | DhT [-], Nc [%]   | PR, eff    |
| NOZZLE     | Core Nozzle, Fan Nozzle | NPR [-]           | Cfg        |
| INLET      | Inlet                   | MN [-]            | eta_r      |

## File Format

### Header Fields (all map types)
```
TYPE        COMPRESSOR | TURBINE | NOZZLE | INLET
NAME        component identifier string
NC_UNITS    PERCENT (Phase 3) | RPM (Phase 4)
SOURCE      data provenance description
DESIGN_PT   type-specific fields (see below)
```

### DESIGN_PT fields by type
```
COMPRESSOR: DESIGN_PT Wc=X   Nc=X   PR=X   eff=X
TURBINE:    DESIGN_PT DhT=X  Nc=X   PR=X   eff=X
NOZZLE:     DESIGN_PT NPR=X  Cfg=X
INLET:      DESIGN_PT MN=X   eta_r=X
```

### 2D Map Block (COMPRESSOR, TURBINE)

Speed lines must be listed in ascending Nc order.
Each speed line requires at least 2 points for interpolation.
Points within a speed line must be listed in ascending x order.
```
SPEED_LINE  Nc=X
  x=V1  PR=V2  eff=V3
  ...
END_SPEED_LINE
```

### VSV Schedule Block (COMPRESSOR only, optional)
```
VSV_SCHEDULE
  Nc=X  angle=Y
  ...
END_VSV_SCHEDULE
```

Negative angle = vanes closed (part power).
Zero angle = design position.
Positive angle = vanes open (not typical).

### 1D Map Block (NOZZLE, INLET)

Points must be listed in ascending x order.
At least 2 points required for interpolation.
```
MAP_1D
  x=V1  y=V2
  ...
END_MAP_1D
```

### Comments and blank lines

Lines starting with `#` are ignored. Blank lines are ignored.

## Units

| Parameter | Units         | Notes                                     |
|-----------|---------------|-------------------------------------------|
| Wc        | kg/s          | Corrected mass flow                       |
| DhT       | -             | Cp*(Tt_in-Tt_exit)/Tt_in — NOT dHt [J/kg] |
| Nc        | % (Phase 3)   | Percent of design corrected speed         |
| Nc        | RPM (Phase 4) | Corrected shaft speed                     |
| PR        | -             | Total pressure ratio                      |
| eff       | -             | Isentropic efficiency                     |
| NPR       | -             | Nozzle pressure ratio Pt_in/Ps_amb        |
| Cfg       | -             | Nozzle velocity coefficient               |
| MN        | -             | Flight Mach number                        |
| eta_r     | -             | Inlet total pressure recovery             |
| angle     | deg           | VSV angle (0 = design, negative = closed) |

## Replacing with real engine data

1. Keep the same file format — only the numbers change
2. Update `NC_UNITS` from `PERCENT` to `RPM` when physical shaft
   speeds are available (Phase 4)
3. Update `SOURCE` field to identify the real data source
4. The C++ map classes do not need modification

## Phase roadmap

- **Phase 3** — `NC_UNITS PERCENT`, synthetic maps scaled from design point
- **Phase 4** — `NC_UNITS RPM`, real engine maps with physical shaft speeds