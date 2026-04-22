#include "CombustorMap.h"
#include <fstream>
#include <sstream>
#include <iostream>

/*
 * CombustorMap.cpp
 * ----------------
 * Implements CombustorMap — loads and interpolates 1D combustor maps.
 *
 * FILE PARSING:
 *   Reads EFF_B_TABLE and DPQP_TABLE blocks directly.
 *   Each data line contains two KEY=VALUE tokens.
 *   EFF_B_TABLE: FAR=X  eff_b=X
 *   DPQP_TABLE:  Wc=X   dPqP=X
 *
 * INTERPOLATION:
 *   Linear 1D interpolation on each table independently.
 *   Clamps silently to table bounds for out-of-range queries.
 */

// =========================================================================
// Constructor — parses the combustor map file
// =========================================================================

CombustorMap::CombustorMap(const std::string& filepath)
{
    std::ifstream file(filepath);
    if (!file.is_open())
    {
        std::cerr << "[CombustorMap] ERROR: cannot open '" << filepath << "'\n";
        return;
    }

    enum State { HEADER, EFF_B, DPQP };
    State state = HEADER;
    bool  type_ok = false;

    std::string raw_line;
    while (std::getline(file, raw_line))
    {
        // Strip inline comments
        const auto hash = raw_line.find('#');
        if (hash != std::string::npos)
            raw_line = raw_line.substr(0, hash);

        // Tokenise
        std::istringstream iss(raw_line);
        std::vector<std::string> tokens;
        std::string tok;
        while (iss >> tok)
            tokens.push_back(tok);
        if (tokens.empty()) continue;

        const std::string& kw = tokens[0];

        // Helper — extract double after '=' in a "KEY=VALUE" token
        auto val = [](const std::string& t) -> double {
            const auto pos = t.find('=');
            if (pos == std::string::npos || pos + 1 >= t.size()) return 0.0;
            try { return std::stod(t.substr(pos + 1)); }
            catch (...) { return 0.0; }
        };

        if (state == HEADER)
        {
            if (kw == "TYPE")
            {
                if (tokens.size() >= 2 && tokens[1] == "COMBUSTOR")
                    type_ok = true;
                else {
                    std::cerr << "[CombustorMap] ERROR: TYPE must be COMBUSTOR"
                              << " in '" << filepath << "'\n";
                    return;
                }
            }
            else if (kw == "EFF_B_TABLE") state = EFF_B;
            else if (kw == "DPQP_TABLE")  state = DPQP;
            // NAME, SOURCE — ignore
        }
        else if (state == EFF_B)
        {
            if (kw == "END_EFF_B_TABLE") { state = HEADER; continue; }

            // Expect: FAR=X  eff_b=X
            MapReader::Map1DPoint pt;
            for (const auto& t : tokens)
            {
                const auto pos = t.find('=');
                if (pos == std::string::npos) continue;
                const std::string key = t.substr(0, pos);
                if      (key == "FAR")   pt.x = val(t);
                else if (key == "eff_b") pt.y = val(t);
            }
            eff_b_table_.push_back(pt);
        }
        else if (state == DPQP)
        {
            if (kw == "END_DPQP_TABLE") { state = HEADER; continue; }

            // Expect: Wc=X  dPqP=X
            MapReader::Map1DPoint pt;
            for (const auto& t : tokens)
            {
                const auto pos = t.find('=');
                if (pos == std::string::npos) continue;
                const std::string key = t.substr(0, pos);
                if      (key == "Wc")   pt.x = val(t);
                else if (key == "dPqP") pt.y = val(t);
            }
            dpqp_table_.push_back(pt);
        }
    }

    if (!type_ok)
    {
        std::cerr << "[CombustorMap] ERROR: TYPE field missing in '"
                  << filepath << "'\n";
        return;
    }
    if (eff_b_table_.size() < 2)
        std::cerr << "[CombustorMap] WARNING: EFF_B_TABLE has fewer than 2 points"
                  << " in '" << filepath << "'\n";
    if (dpqp_table_.size() < 2)
        std::cerr << "[CombustorMap] WARNING: DPQP_TABLE has fewer than 2 points"
                  << " in '" << filepath << "'\n";

    loaded_ = true;
}

// =========================================================================
// interpolate1D
// =========================================================================

double CombustorMap::interpolate1D(
    const std::vector<MapReader::Map1DPoint>& table,
    double                                     x) const noexcept
{
    if (table.empty())             return 0.0;
    if (x <= table.front().x)      return table.front().y;
    if (x >= table.back().x)       return table.back().y;

    for (std::size_t i = 0; i + 1 < table.size(); ++i)
    {
        if (x >= table[i].x && x <= table[i+1].x)
        {
            const double t = (x - table[i].x) / (table[i+1].x - table[i].x);
            return table[i].y + t * (table[i+1].y - table[i].y);
        }
    }
    return table.back().y;
}

// =========================================================================
// lookup
// =========================================================================

CombustorMapResult CombustorMap::lookup(double FAR, double Wc_in) const noexcept
{
    CombustorMapResult result;   // defaults: eff_b=0.99, dPqP=0.04

    if (!loaded_) return result;

    if (eff_b_table_.size() >= 2)
        result.eff_b = interpolate1D(eff_b_table_, FAR);

    if (dpqp_table_.size() >= 2)
        result.dPqP  = interpolate1D(dpqp_table_, Wc_in);

    return result;
}