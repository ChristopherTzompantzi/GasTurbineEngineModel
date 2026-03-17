#include "MapReader.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>

/*
 * MapReader.cpp
 * -------------
 * Implements the shared map file parser for all engine performance maps.
 *
 * PARSE SEQUENCE:
 * 1. Open file — report error and return empty MapFileData if not found
 * 2. Read line by line — strip comments and blank lines
 * 3. Parse header fields — TYPE, NAME, NC_UNITS, SOURCE, DESIGN_PT
 * 4. Parse VSV_SCHEDULE block (COMPRESSOR only)
 * 5. Parse SPEED_LINE blocks (COMPRESSOR, TURBINE)
 * 6. Parse MAP_1D block (NOZZLE, INLET)
 * 7. Validate — check minimum point counts, required fields present
 * 8. Return populated MapFileData
 *
 * ERROR HANDLING:
 * All errors are reported to stderr with the filename and line number.
 * On error, read() returns a MapFileData with empty type field.
 * Callers check data.type.empty() to detect failure.
 */

namespace MapReader {

// =========================================================================
// parseLine
//
// Splits a raw file line into whitespace-separated tokens.
// Returns empty vector for:
//   - Lines starting with # (comments)
//   - Blank lines
//   - Lines containing only whitespace
// =========================================================================

std::vector<std::string> parseLine(const std::string& line)
{
    std::vector<std::string> tokens;

    // Strip leading whitespace to check for comment or blank
    std::string trimmed = line;
    trimmed.erase(0, trimmed.find_first_not_of(" \t\r\n"));

    // Skip comments and blank lines
    if (trimmed.empty() || trimmed[0] == '#')
        return tokens;

    // Split on whitespace
    std::istringstream iss(trimmed);
    std::string token;
    while (iss >> token)
        tokens.push_back(token);

    return tokens;
}

// =========================================================================
// parseKeyValue
//
// Extracts a double from a "KEY=VALUE" token.
// The key argument is used only for error reporting.
//
// Example: parseKeyValue("Wc=33.33", "Wc") returns 33.33
// =========================================================================

double parseKeyValue(const std::string& token, const std::string& key)
{
    const auto pos = token.find('=');
    if (pos == std::string::npos || pos + 1 >= token.size())
    {
        std::cerr << "[MapReader] ERROR: malformed token '" << token
                  << "' — expected KEY=VALUE format for key '" << key << "'\n";
        return 0.0;
    }
    try {
        std::string val = token.substr(pos + 1);
        // Strip leading whitespace — generated map files may have spaces after =
        val.erase(0, val.find_first_not_of(" \t"));
        return std::stod(val);
    }
    catch (...) {
        std::cerr << "[MapReader] ERROR: cannot parse value in token '"
                  << token << "' for key '" << key << "'\n";
        return 0.0;
    }
}

// =========================================================================
// parseDesignPoint
//
// Parses the DESIGN_PT header line. Field interpretation depends on
// map type:
//
//   COMPRESSOR: DESIGN_PT Wc=X  Nc=X  PR=X  eff=X
//   TURBINE:    DESIGN_PT DhT=X Nc=X  PR=X  eff=X
//   NOZZLE:     DESIGN_PT NPR=X Cfg=X
//   INLET:      DESIGN_PT MN=X  eta_r=X
// =========================================================================

MapDesignPoint parseDesignPoint(const std::vector<std::string>& tokens,
                                const std::string&             /*type*/)
{
    MapDesignPoint dp;

    // tokens[0] = "DESIGN_PT", tokens[1..n] = key=value pairs
    for (std::size_t i = 1; i < tokens.size(); ++i)
    {
        const std::string& t = tokens[i];
        const auto pos = t.find('=');
        if (pos == std::string::npos) continue;

        const std::string key = t.substr(0, pos);

        if      (key == "Wc"    || key == "DhT" ||
                 key == "NPR"   || key == "MN")
            dp.x_design = parseKeyValue(t, key);
        else if (key == "Nc")
            dp.Nc        = parseKeyValue(t, key);
        else if (key == "PR")
            dp.PR        = parseKeyValue(t, key);
        else if (key == "eff")
            dp.eff       = parseKeyValue(t, key);
        else if (key == "Cfg"   || key == "eta_r")
            dp.y_design  = parseKeyValue(t, key);
    }

    return dp;
}

// =========================================================================
// read
//
// Main entry point — parses a complete map file and returns MapFileData.
//
// Reads the file in a single pass. Maintains a simple state machine:
//   STATE_HEADER      — reading header fields
//   STATE_VSV         — inside VSV_SCHEDULE...END_VSV_SCHEDULE block
//   STATE_SPEEDLINE   — inside SPEED_LINE...END_SPEED_LINE block
//   STATE_MAP1D       — inside MAP_1D...END_MAP_1D block
// =========================================================================

MapFileData read(const std::string& filepath)
{
    MapFileData data;

    // Step 1 — Open file
    std::ifstream file(filepath);
    if (!file.is_open())
    {
        std::cerr << "[MapReader] ERROR: cannot open file '" << filepath << "'\n";
        return data;   // data.type is empty — signals failure to caller
    }

    // Parser state
    enum State { STATE_HEADER, STATE_VSV, STATE_SPEEDLINE, STATE_MAP1D };
    State state = STATE_HEADER;

    MapSpeedLine current_speedline;
    int          line_num = 0;

    std::string raw_line;
    while (std::getline(file, raw_line))
    {
        ++line_num;
        const auto tokens = parseLine(raw_line);
        if (tokens.empty()) continue;   // blank or comment

        const std::string& keyword = tokens[0];

        // ----------------------------------------------------------------
        // State: HEADER — reading top-level header fields
        // ----------------------------------------------------------------
        if (state == STATE_HEADER)
        {
            if (keyword == "TYPE")
            {
                if (tokens.size() >= 2) data.type = tokens[1];
            }
            else if (keyword == "NAME")
            {
                if (tokens.size() >= 2) data.name = tokens[1];
            }
            else if (keyword == "NC_UNITS")
            {
                if (tokens.size() >= 2) data.nc_units = tokens[1];
            }
            else if (keyword == "SOURCE")
            {
                // SOURCE may contain multiple words — join them
                for (std::size_t i = 1; i < tokens.size(); ++i)
                {
                    if (i > 1) data.source += " ";
                    data.source += tokens[i];
                }
            }
            else if (keyword == "DESIGN_PT")
            {
                data.design_pt = parseDesignPoint(tokens, data.type);
            }
            else if (keyword == "VSV_SCHEDULE")
            {
                state = STATE_VSV;
            }
            else if (keyword == "SPEED_LINE")
            {
                // Begin a new speed line block
                current_speedline = MapSpeedLine{};
                // Parse Nc from "SPEED_LINE Nc=X"
                if (tokens.size() >= 2)
                    current_speedline.Nc = parseKeyValue(tokens[1], "Nc");
                state = STATE_SPEEDLINE;
            }
            else if (keyword == "MAP_1D")
            {
                state = STATE_MAP1D;
            }
            else
            {
                std::cerr << "[MapReader] WARNING: unknown keyword '"
                          << keyword << "' at line " << line_num
                          << " in '" << filepath << "' — ignored\n";
            }
        }

        // ----------------------------------------------------------------
        // State: VSV_SCHEDULE block
        // ----------------------------------------------------------------
        else if (state == STATE_VSV)
        {
            if (keyword == "END_VSV_SCHEDULE")
            {
                state = STATE_HEADER;
            }
            else
            {
                // Expect: Nc=X  angle=X
                double Nc    = 0.0;
                double angle = 0.0;
                for (std::size_t i = 0; i < tokens.size(); ++i)
                {
                    const auto pos = tokens[i].find('=');
                    if (pos == std::string::npos) continue;
                    const std::string key = tokens[i].substr(0, pos);
                    if      (key == "Nc")    Nc    = parseKeyValue(tokens[i], "Nc");
                    else if (key == "angle") angle = parseKeyValue(tokens[i], "angle");
                }
                data.vsv_schedule.push_back({Nc, angle});
            }
        }

        // ----------------------------------------------------------------
        // State: SPEED_LINE block
        // ----------------------------------------------------------------
        else if (state == STATE_SPEEDLINE)
        {
            if (keyword == "END_SPEED_LINE")
            {
                // Validate minimum points for interpolation
                if (current_speedline.points.size() < 2)
                {
                    std::cerr << "[MapReader] WARNING: SPEED_LINE Nc="
                              << current_speedline.Nc
                              << " has fewer than 2 points — interpolation"
                              << " will fail at line " << line_num
                              << " in '" << filepath << "'\n";
                }
                data.speed_lines.push_back(current_speedline);
                state = STATE_HEADER;
            }
            else
            {
                // Expect data points — format depends on map type:
                //   COMPRESSOR: Wc=X  PR=X  eff=X
                //   TURBINE:    DhT=X PR=X  eff=X
                MapDataPoint pt;
                for (std::size_t i = 0; i < tokens.size(); ++i)
                {
                    const auto pos = tokens[i].find('=');
                    if (pos == std::string::npos) continue;
                    const std::string key = tokens[i].substr(0, pos);
                    if      (key == "Wc"  || key == "DhT")
                        pt.x  = parseKeyValue(tokens[i], key);
                    else if (key == "PR")
                        pt.y1 = parseKeyValue(tokens[i], key);
                    else if (key == "eff")
                        pt.y2 = parseKeyValue(tokens[i], key);
                }
                current_speedline.points.push_back(pt);
            }
        }

        // ----------------------------------------------------------------
        // State: MAP_1D block
        // ----------------------------------------------------------------
        else if (state == STATE_MAP1D)
        {
            if (keyword == "END_MAP_1D")
            {
                // Validate minimum points
                if (data.map_1d.size() < 2)
                {
                    std::cerr << "[MapReader] WARNING: MAP_1D has fewer"
                              << " than 2 points in '" << filepath << "'"
                              << " — interpolation will fail\n";
                }
                state = STATE_HEADER;
            }
            else
            {
                // Expect: X=value Y=value
                // NOZZLE: NPR=X Cfg=X
                // INLET:  MN=X  eta_r=X
                Map1DPoint pt;
                for (std::size_t i = 0; i < tokens.size(); ++i)
                {
                    const auto pos = tokens[i].find('=');
                    if (pos == std::string::npos) continue;
                    const std::string key = tokens[i].substr(0, pos);
                    if      (key == "NPR"   || key == "MN")
                        pt.x = parseKeyValue(tokens[i], key);
                    else if (key == "Cfg"   || key == "eta_r")
                        pt.y = parseKeyValue(tokens[i], key);
                }
                data.map_1d.push_back(pt);
            }
        }
    }

    // Step 7 — Validate required header fields
    if (data.type.empty())
    {
        std::cerr << "[MapReader] ERROR: TYPE field missing in '"
                  << filepath << "'\n";
        return MapFileData{};
    }
    if (data.name.empty())
    {
        std::cerr << "[MapReader] WARNING: NAME field missing in '"
                  << filepath << "'\n";
    }
    if (data.nc_units.empty())
    {
        std::cerr << "[MapReader] WARNING: NC_UNITS field missing in '"
                  << filepath << "' — defaulting to PERCENT\n";
        data.nc_units = "PERCENT";
    }

    // Validate 2D maps have at least 2 speed lines for interpolation
    if ((data.type == "COMPRESSOR" || data.type == "TURBINE") &&
        data.speed_lines.size() < 2)
    {
        std::cerr << "[MapReader] ERROR: " << data.type << " map '"
                  << filepath << "' has fewer than 2 speed lines"
                  << " — interpolation requires at least 2\n";
        return MapFileData{};
    }

    // Validate 1D maps have at least 2 points
    if ((data.type == "NOZZLE" || data.type == "INLET") &&
        data.map_1d.size() < 2)
    {
        std::cerr << "[MapReader] ERROR: " << data.type << " map '"
                  << filepath << "' has fewer than 2 points"
                  << " — interpolation requires at least 2\n";
        return MapFileData{};
    }

    return data;
}

}   // namespace MapReader