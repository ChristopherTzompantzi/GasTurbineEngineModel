#ifndef MAPREADER_H
#define MAPREADER_H

#include <string>
#include <vector>

/*
 * MapReader.h
 * -----------
 * Declares the MapReader utility — shared file parser for all engine
 * performance map files in the GasTurbineEngineModel.
 *
 * PURPOSE:
 *   Centralises all map file I/O in one place. All map classes
 *   (CompressorMap, TurbineMap, NozzleMap, InletMap) call MapReader
 *   to load their data files. If the file format changes, only
 *   MapReader needs updating — not each individual map class.
 *
 * SUPPORTED MAP TYPES:
 *   COMPRESSOR — 2D map: speed lines of (Wc, PR, eff) vs Nc%
 *                        with optional VSV schedule
 *   TURBINE    — 2D map: speed lines of (DhT, PR, eff) vs Nc%
 *                        DhT = Cp*(Tt_in - Tt_exit)/Tt_in [-]
 *   NOZZLE     — 1D map: Cfg vs NPR
 *   INLET      — 1D map: eta_r vs MN
 *
 * FILE FORMAT:
 *   See maps/README.md for the complete file format specification.
 *   Key structure:
 *     # comments
 *     TYPE        COMPRESSOR | TURBINE | NOZZLE | INLET
 *     NAME        component identifier
 *     NC_UNITS    PERCENT | RPM
 *     SOURCE      data provenance description
 *     DESIGN_PT   type-specific design point fields
 *     VSV_SCHEDULE ... END_VSV_SCHEDULE   (COMPRESSOR only)
 *     SPEED_LINE Nc=X ... END_SPEED_LINE  (COMPRESSOR, TURBINE)
 *     MAP_1D     ... END_MAP_1D           (NOZZLE, INLET)
 *
 * NPSS ALIGNMENT:
 *   Matches NPSS convention — map data is stored in external files,
 *   parsed at construction time, and cached in memory for fast lookup.
 *   NC_UNITS PERCENT is used for Phase 3 (design point analysis).
 *   NC_UNITS RPM will be used in Phase 4 (off-design with shaft speeds).
 *
 * UNITS:
 *   Wc    [kg/s corrected]
 *   DhT   [-] dimensionless energy function
 *   Nc    [% of design corrected speed] (Phase 3)
 *   PR    [-]
 *   eff   [-]
 *   NPR   [-]
 *   MN    [-]
 *   Cfg   [-]
 *   eta_r [-]
 */

namespace MapReader {

// =========================================================================
// DATA STRUCTURES
// =========================================================================

/*
 * MapDataPoint — single operating point on a speed line.
 *
 * For COMPRESSOR: x = Wc [kg/s corrected]
 * For TURBINE:    x = DhT [-] (dimensionless energy function)
 *                 NOTE: DhT [-] is the dimensionless map ordinate
 *                 (Cp*(Tt_in - Tt_exit)/Tt_in). This is NOT the same
 *                 as dHt [J/kg] (dimensional specific work) used in
 *                 Turbine.cpp for shaft power calculation.
 * y1 = PR [-], y2 = eff [-]
 */
struct MapDataPoint {
    double x  = 0.0;   // Wc [kg/s] or DhT [-]
    double y1 = 0.0;   // pressure ratio PR [-]
    double y2 = 0.0;   // isentropic efficiency eff [-]
};

/*
 * MapSpeedLine — one speed line at constant corrected speed Nc.
 *
 * Contains an ordered sequence of MapDataPoints from low to high x.
 * vsv_angle is interpolated from the VSV schedule at this Nc —
 * 0.0 if no VSV schedule is present.
 */
struct MapSpeedLine {
    double                   Nc        = 0.0;   // corrected speed [% or RPM]
    double                   vsv_angle = 0.0;   // VSV angle [deg]
    std::vector<MapDataPoint> points;            // operating points on this line
};

/*
 * Map1DPoint — single point on a 1D performance map.
 *
 * For NOZZLE: x = NPR [-], y = Cfg [-]
 * For INLET:  x = MN  [-], y = eta_r [-]
 *             (eta_r is the total-pressure recovery factor,
 *              equivalent to recovery_factor_ in Inlet.cpp)
 */
struct Map1DPoint {
    double x = 0.0;   // independent variable
    double y = 0.0;   // dependent variable
};

/*
 * MapDesignPoint — design point reference values from file header.
 *
 * Used by map classes to verify the loaded data is consistent with
 * the engine design point and to set normalisation references.
 *
 * Fields populated depend on map type:
 *   COMPRESSOR: Nc, x_design (=Wc_d), PR, eff
 *   TURBINE:    Nc, x_design (=DhT_d), PR, eff
 *   NOZZLE:     x_design (=NPR_d), y_design (=Cfg_d)
 *   INLET:      x_design (=MN_d),  y_design (=eta_r_d)
 */
struct MapDesignPoint {
    double Nc       = 0.0;   // design corrected speed [% or RPM]
    double x_design = 0.0;   // design Wc, DhT, NPR, or MN
    double PR       = 0.0;   // design pressure ratio [-]
    double eff      = 0.0;   // design isentropic efficiency [-]
    double y_design = 0.0;   // design Cfg or eta_r (1D maps only)
};

/*
 * VsvPoint — one entry in a VSV (Variable Stator Vane) schedule.
 *
 * Maps corrected speed Nc to stator vane angle. Used by CompressorMap
 * to interpolate vsv_angle onto each MapSpeedLine after loading.
 */
struct VsvPoint {
    double Nc    = 0.0;   // corrected speed [% or RPM]
    double angle = 0.0;   // VSV angle [deg]
};

/*
 * MapFileData — complete parsed contents of one map file.
 *
 * Returned by MapReader::read(). The calling map class interprets
 * the contents based on the type field.
 *
 * speed_lines — populated for COMPRESSOR and TURBINE maps
 * map_1d      — populated for NOZZLE and INLET maps
 * vsv_schedule— populated for COMPRESSOR maps with VSV_SCHEDULE block
 */
struct MapFileData {
    std::string              type;          // COMPRESSOR, TURBINE, NOZZLE, INLET
    std::string              name;          // component identifier
    std::string              nc_units;      // PERCENT or RPM
    std::string              source;        // data provenance
    MapDesignPoint           design_pt;     // design point reference

    std::vector<MapSpeedLine> speed_lines;   // 2D map data
    std::vector<Map1DPoint>   map_1d;        // 1D map data
    std::vector<VsvPoint>     vsv_schedule;  // VSV schedule (COMPRESSOR only)
};

// =========================================================================
// PUBLIC INTERFACE
// =========================================================================

/*
 * read — parses a map file and returns its contents as MapFileData.
 *
 * Reads the file line by line. Strips comments and blank lines.
 * Parses header fields, VSV_SCHEDULE, SPEED_LINE, and MAP_1D blocks.
 * Reports errors to stderr with line numbers for easy debugging.
 *
 * @param filepath   Path to the map file (absolute or relative to executable)
 * @return           MapFileData containing all parsed map data.
 *                   On error: returns MapFileData with empty type field.
 *                   Caller should check data.type.empty() for error detection.
 *
 * ERRORS REPORTED TO STDERR:
 *   - File not found
 *   - Missing required header fields (TYPE, NAME, DESIGN_PT)
 *   - Malformed SPEED_LINE or MAP_1D blocks
 *   - Unknown keywords
 *   - Insufficient points on a speed line (minimum 2 required for interpolation)
 */
MapFileData read(const std::string& filepath);

// =========================================================================
// INTERNAL HELPERS — not intended for direct use by map classes
// =========================================================================

/*
 * parseLine — splits a line into keyword and value tokens.
 * Strips leading/trailing whitespace. Returns empty vector for
 * comment lines (starting with #) and blank lines.
 */
std::vector<std::string> parseLine(const std::string& line);

/*
 * parseKeyValue — extracts a double value from a "KEY=VALUE" token.
 * Returns 0.0 and reports error if token is malformed.
 *
 * Example: parseKeyValue("Wc=33.33") returns 33.33
 */
double parseKeyValue(const std::string& token, const std::string& key);

/*
 * parseDesignPoint — parses the DESIGN_PT header line for each map type.
 * Interprets fields based on the TYPE already parsed from the header.
 */
MapDesignPoint parseDesignPoint(const std::vector<std::string>& tokens,
                                const std::string&             /*type*/);

}   // namespace MapReader

#endif // MAPREADER_H