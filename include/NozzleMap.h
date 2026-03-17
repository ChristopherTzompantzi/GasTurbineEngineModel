#ifndef NOZZLEMAP_H
#define NOZZLEMAP_H

#include "MapReader.h"
#include <string>
#include <vector>

/*
 * NozzleMap.h
 * -----------
 * Declares the NozzleMap class — loads and interpolates 1D nozzle
 * discharge coefficient maps from .map files.
 *
 * USED BY:
 *   Nozzle    — core nozzle (hot stream)
 *   FanNozzle — bypass nozzle (cold stream)
 *   Same class instantiated twice with different map files.
 *
 * MAP AXES:
 *   x-axis: NPR [-] — nozzle pressure ratio = Pt_inlet / Ps_ambient
 *   output: Cfg [-] — velocity coefficient / discharge coefficient
 *
 * INTERPOLATION — linear:
 *   Linear interpolation between adjacent NPR points.
 *   Silently clamped to map bounds if NPR is out of range.
 *
 * FUTURE WORK (Phase 5):
 *   Variable area nozzle scheduling — Cfg as function of NPR and Ath setting.
 *
 * UNITS:
 *   NPR  [-]
 *   Cfg  [-]
 *
 * NPSS ALIGNMENT:
 *   Mirrors NPSS NozzleMap element — Cfg vs NPR 1D table lookup.
 */

class NozzleMap {
public:

    /*
     * Constructor — loads and parses the map file.
     * @param filepath  Path to the .map file
     */
    explicit NozzleMap(const std::string& filepath);

    /*
     * isLoaded — returns true if the map file parsed successfully.
     */
    bool isLoaded() const noexcept { return loaded_; }

    /*
     * lookup — returns Cfg at the given NPR.
     * @param NPR  Nozzle pressure ratio [-]
     * @return     Cfg [-]
     */
    double lookup(double NPR) const;

    /*
     * designPoint — returns design point from map file header.
     */
    const MapReader::MapDesignPoint& designPoint() const noexcept
    {
        return design_pt_;
    }

private:

    bool                          loaded_    = false;   // true if map parsed successfully
    MapReader::MapDesignPoint     design_pt_;           // design point from file header
    std::vector<MapReader::Map1DPoint> points_;         // Cfg vs NPR data points

    /*
     * interpolate — linear interpolation between two adjacent NPR points.
     * Silently clamps if NPR is outside map range.
     */
    double interpolate(double NPR) const noexcept;
};

#endif // NOZZLEMAP_H