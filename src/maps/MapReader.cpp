#include "MapReader.h"

#include <fstream>
#include <sstream>
#include <iostream>

/*
 * MapReader.cpp
 * -------------
 * Implements the shared map file parser for all engine performance maps.
 *
 * PARSE SEQUENCE:
 * 1. Open file — report error and return empty MapFileData if not found
 * 2. Read line by line — strip comments and blank lines
 * 3. Parse header fields — TYPE, NAME, NC_UNITS, SOURCE, SCALE_*, DESIGN_PT
 * 4. Parse VSV_SCHEDULE / IGV_SCHEDULE block (COMPRESSOR/FAN only)
 * 5. Parse SURGE_LINE block (COMPRESSOR/FAN only)
 * 6. Parse SPEED_LINE blocks (COMPRESSOR, TURBINE)
 * 7. Parse MAP_1D block (NOZZLE, INLET)
 * 8. Validate — check minimum point counts, required fields present
 * 9. Return populated MapFileData
 *
 * SCALE FACTORS (Phase 5 — Kurzke-Riegler):
 *   SCALE_PR, SCALE_EFF, SCALE_WC, SCALE_NC parsed from header.
 *   All default to 1.0 if absent — backward compatible with Phase 3/4 maps.
 *   Applied at lookup time in CompressorMap and TurbineMap, not here.
 *
 * IGV_SCHEDULE:
 *   Treated identically to VSV_SCHEDULE — stored in vsv_schedule field.
 *   Fan maps use IGV_SCHEDULE keyword; compressor maps use VSV_SCHEDULE.
 *   Both populate the same data structure.
 *
 * SURGE_LINE:
 *   Stores (Wc, PR_surge) pairs defining the surge boundary.
 *   Parsed identically to MAP_1D but stored in surge_line field.
 *   Scaled at lookup time by SCALE_WC and SCALE_PR in CompressorMap.
 */

namespace MapReader {

// =========================================================================
// parseLine
// =========================================================================

std::vector<std::string> parseLine(const std::string& line)
{
    std::vector<std::string> tokens;

    std::string trimmed = line;
    trimmed.erase(0, trimmed.find_first_not_of(" \t\r\n"));

    if (trimmed.empty() || trimmed[0] == '#')
        return tokens;

    std::istringstream iss(trimmed);
    std::string token;
    while (iss >> token)
        tokens.push_back(token);

    return tokens;
}

// =========================================================================
// parseKeyValue
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
// =========================================================================

MapDesignPoint parseDesignPoint(const std::vector<std::string>& tokens,
                                const std::string& /*type*/)
{
    MapDesignPoint dp;

    for (std::size_t i = 1; i < tokens.size(); ++i)
    {
        const std::string& t = tokens[i];
        const auto pos = t.find('=');
        if (pos == std::string::npos) continue;

        const std::string key = t.substr(0, pos);

        if      (key == "Wc"  || key == "DhT" ||
                 key == "NPR" || key == "MN")
            dp.x_design = parseKeyValue(t, key);
        else if (key == "Nc")
            dp.Nc        = parseKeyValue(t, key);
        else if (key == "PR")
            dp.PR        = parseKeyValue(t, key);
        else if (key == "eff")
            dp.eff       = parseKeyValue(t, key);
        else if (key == "Cfg" || key == "eta_r")
            dp.y_design  = parseKeyValue(t, key);
    }

    return dp;
}

// =========================================================================
// read
// =========================================================================

MapFileData read(const std::string& filepath)
{
    MapFileData data;

    // Step 1 — Open file
    std::ifstream file(filepath);
    if (!file.is_open())
    {
        std::cerr << "[MapReader] ERROR: cannot open file '" << filepath << "'\n";
        return data;
    }

    // Parser state machine
    enum State {
        STATE_HEADER,
        STATE_VSV,
        STATE_SURGELINE,
        STATE_SPEEDLINE,
        STATE_MAP1D
    };
    State state = STATE_HEADER;

    MapSpeedLine current_speedline;
    int          line_num = 0;

    std::string raw_line;
    while (std::getline(file, raw_line))
    {
        ++line_num;
        const auto tokens = parseLine(raw_line);
        if (tokens.empty()) continue;

        const std::string& keyword = tokens[0];

        // ----------------------------------------------------------------
        // STATE_HEADER
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
                for (std::size_t i = 1; i < tokens.size(); ++i)
                {
                    if (i > 1) data.source += " ";
                    data.source += tokens[i];
                }
            }
            // --- Phase 5: Kurzke-Riegler scale factors ---
            else if (keyword == "SCALE_PR")
            {
                if (tokens.size() >= 2)
                    data.scales.scale_PR = std::stod(tokens[1]);
            }
            else if (keyword == "SCALE_EFF")
            {
                if (tokens.size() >= 2)
                    data.scales.scale_Eff = std::stod(tokens[1]);
            }
            else if (keyword == "SCALE_WC")
            {
                if (tokens.size() >= 2)
                    data.scales.scale_Wc = std::stod(tokens[1]);
            }
            else if (keyword == "SCALE_NC")
            {
                if (tokens.size() >= 2)
                    data.scales.scale_Nc = std::stod(tokens[1]);
            }
            // --- Design point ---
            else if (keyword == "DESIGN_PT")
            {
                data.design_pt = parseDesignPoint(tokens, data.type);
            }
            // --- VSV_SCHEDULE (compressor) or IGV_SCHEDULE (fan — alias) ---
            else if (keyword == "VSV_SCHEDULE" || keyword == "IGV_SCHEDULE")
            {
                state = STATE_VSV;
            }
            // --- SURGE_LINE block (compressor and fan) ---
            else if (keyword == "SURGE_LINE")
            {
                state = STATE_SURGELINE;
            }
            else if (keyword == "SPEED_LINE")
            {
                current_speedline = MapSpeedLine{};
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
        // STATE_VSV — VSV_SCHEDULE or IGV_SCHEDULE block
        // ----------------------------------------------------------------
        else if (state == STATE_VSV)
        {
            if (keyword == "END_VSV_SCHEDULE" || keyword == "END_IGV_SCHEDULE")
            {
                state = STATE_HEADER;
            }
            else
            {
                // Expect: Nc=X  angle=X
                double Nc    = 0.0;
                double angle = 0.0;
                for (const auto& t : tokens)
                {
                    const auto pos = t.find('=');
                    if (pos == std::string::npos) continue;
                    const std::string key = t.substr(0, pos);
                    if      (key == "Nc")    Nc    = parseKeyValue(t, "Nc");
                    else if (key == "angle") angle = parseKeyValue(t, "angle");
                }
                data.vsv_schedule.push_back({Nc, angle});
            }
        }

        // ----------------------------------------------------------------
        // STATE_SURGELINE — SURGE_LINE block
        // Stores (Wc, PR_surge) pairs defining the surge boundary.
        // Surge line is stored unscaled — scaling applied at lookup time.
        // ----------------------------------------------------------------
        else if (state == STATE_SURGELINE)
        {
            if (keyword == "END_SURGE_LINE")
            {
                if (data.surge_line.size() < 2)
                {
                    std::cerr << "[MapReader] WARNING: SURGE_LINE has fewer"
                              << " than 2 points in '" << filepath
                              << "' — surge margin lookup will fail\n";
                }
                state = STATE_HEADER;
            }
            else
            {
                // Expect: Wc=X  PR=X
                Map1DPoint pt;
                for (const auto& t : tokens)
                {
                    const auto pos = t.find('=');
                    if (pos == std::string::npos) continue;
                    const std::string key = t.substr(0, pos);
                    if      (key == "Wc") pt.x = parseKeyValue(t, "Wc");
                    else if (key == "PR") pt.y = parseKeyValue(t, "PR");
                }
                data.surge_line.push_back(pt);
            }
        }

        // ----------------------------------------------------------------
        // STATE_SPEEDLINE
        // ----------------------------------------------------------------
        else if (state == STATE_SPEEDLINE)
        {
            if (keyword == "END_SPEED_LINE")
            {
                if (current_speedline.points.size() < 2)
                {
                    std::cerr << "[MapReader] WARNING: SPEED_LINE Nc="
                              << current_speedline.Nc
                              << " has fewer than 2 points at line "
                              << line_num << " in '" << filepath << "'\n";
                }
                data.speed_lines.push_back(current_speedline);
                state = STATE_HEADER;
            }
            else
            {
                MapDataPoint pt;
                for (const auto& t : tokens)
                {
                    const auto pos = t.find('=');
                    if (pos == std::string::npos) continue;
                    const std::string key = t.substr(0, pos);
                    if      (key == "Wc" || key == "DhT")
                        pt.x  = parseKeyValue(t, key);
                    else if (key == "PR")
                        pt.y1 = parseKeyValue(t, key);
                    else if (key == "eff")
                        pt.y2 = parseKeyValue(t, key);
                }
                current_speedline.points.push_back(pt);
            }
        }

        // ----------------------------------------------------------------
        // STATE_MAP1D
        // ----------------------------------------------------------------
        else if (state == STATE_MAP1D)
        {
            if (keyword == "END_MAP_1D")
            {
                if (data.map_1d.size() < 2)
                {
                    std::cerr << "[MapReader] WARNING: MAP_1D has fewer"
                              << " than 2 points in '" << filepath << "'\n";
                }
                state = STATE_HEADER;
            }
            else
            {
                Map1DPoint pt;
                for (const auto& t : tokens)
                {
                    const auto pos = t.find('=');
                    if (pos == std::string::npos) continue;
                    const std::string key = t.substr(0, pos);
                    if      (key == "NPR" || key == "MN")
                        pt.x = parseKeyValue(t, key);
                    else if (key == "Cfg" || key == "eta_r")
                        pt.y = parseKeyValue(t, key);
                }
                data.map_1d.push_back(pt);
            }
        }
    }

    // Step 8 — Validate required fields
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
                  << filepath << "' — defaulting to RPM\n";
        data.nc_units = "RPM";
    }

    if ((data.type == "COMPRESSOR" || data.type == "TURBINE") &&
        data.speed_lines.size() < 2)
    {
        std::cerr << "[MapReader] ERROR: " << data.type << " map '"
                  << filepath << "' has fewer than 2 speed lines\n";
        return MapFileData{};
    }

    if ((data.type == "NOZZLE" || data.type == "INLET") &&
        data.map_1d.size() < 2)
    {
        std::cerr << "[MapReader] ERROR: " << data.type << " map '"
                  << filepath << "' has fewer than 2 points\n";
        return MapFileData{};
    }

    return data;
}

}   // namespace MapReader