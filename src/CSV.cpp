#include "CSV.hpp"
#include <iostream>
#include <iterator>
//---------------------------------------------------------------------------
/* adaptivelindp: Reference implementation of the adaptive LinDP algorithm
 * Copyright (C) 2024 Altan Birler
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
//---------------------------------------------------------------------------
using namespace std;
//---------------------------------------------------------------------------
namespace fastoptim {
//---------------------------------------------------------------------------
CSVBase::CSVBase(std::ostream& out, vector<string> headerInp, string sepInp)
: out(out), headerStrings(move(headerInp)), sep(move(sepInp))
// Constructor
{
    if (!headerStrings.empty()) {
        formatString.reserve(headerStrings.size() * 3 - 1);
        formatString += "{}"sv;
        for (size_t i = 1; i < headerStrings.size(); i++) {
            formatString += sep;
            formatString += "{}"sv;
        }
    }
    formatString += "\n";
}
//---------------------------------------------------------------------------
/// Destructor
CSVBase::~CSVBase() noexcept = default;
//---------------------------------------------------------------------------
void CSVBase::header()
// Write header
{
    {
        unique_lock ul(lock);
        if (!headerStrings.empty()) {
            out << headerStrings[0];
            for (size_t i = 1; i < headerStrings.size(); i++) {
                out << sep;
                out << headerStrings[i];
            }
        }
        out << "\n";
        out << std::flush;
    }
}
//---------------------------------------------------------------------------
void CSVBase::ln(std::format_args args)
// Write line
{
    {
        unique_lock ul(lock);
        vformat_to(ostream_iterator<char>(out), formatString, move(args));
        out.flush();
    }
}
//---------------------------------------------------------------------------
}
//---------------------------------------------------------------------------