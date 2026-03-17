#ifndef INLETMAP_H
#define INLETMAP_H

#include "MapReader.h"
#include <string>
#include <vector>

/*
 * InletMap.h
 * ----------
 * Declares the InletMap class — loads and interpolates 1D inlet
 * total pressure recovery maps from .map files.
 *
 * MAP AXES:
 *   x-axis: MN    [-] — flight Mach number
 *   output: eta_r [-] — total pressure recovery factor
 *                       equivalent to recovery_factor in Inlet.cpp
 *
 * INTERPOLATION — linear between adjacent MN points.
 * EXTRAPOLATION — silently clamped to map bounds.
 *
 * FUTURE WORK (Phase 5):
 *   Angle of attack correction on eta_r.
 *   Supersonic inlet with normal shock recovery model.
 *
 * UNITS:
 *   MN    [-]
 *   eta_r [-]
 *
 * NPSS ALIGNMENT:
 *   Mirrors NPSS InletMap element — eta_r vs MN 1D table lookup.
 */

class InletMap {
public:

    /*
     * Constructor — loads and parses the map file.
     * Call isLoaded() after construction to verify success.
     *
     * @param filepath  Path to the .map file
     */
    explicit InletMap(const std::string& filepath);

    /*
     * isLoaded — returns true if the map file was parsed successfully.
     * Elements must check this before calling lookup().
     */
    bool isLoaded() const noexcept { return loaded_; }

    /*
     * lookup — returns eta_r at the given flight Mach number.
     *
     * @param MN  Flight Mach number [-]
     * @return    eta_r [-] total pressure recovery factor
     */
    double lookup(double MN) const;

    /*
     * designPoint — returns the design point from the map file header.
     * Used by elements to verify consistency with cycle design point.
     */
    const MapReader::MapDesignPoint& designPoint() const noexcept
    {
        return design_pt_;
    }

private:

    bool                               loaded_    = false;   // true if map parsed successfully
    MapReader::MapDesignPoint          design_pt_;           // design point from file header
    std::vector<MapReader::Map1DPoint> points_;              // eta_r vs MN data points

    /*
     * interpolate — linear interpolation of eta_r at the given MN.
     * Silently clamps to map bounds if MN is out of range.
     *
     * @param MN  Flight Mach number [-]
     * @return    eta_r [-]
     */
    double interpolate(double MN) const noexcept;
};

#endif // INLETMAP_H