#pragma once
//---------------------------------------------------------------------------
#include <iosfwd>
#include <format>
#include <string>
#include <vector>
#include <mutex>
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
namespace fastoptim {
//---------------------------------------------------------------------------
/// Utility for writing csvs
class CSVBase {
    /// Stream
    std::ostream& out;
    /// Header
    std::vector<std::string> headerStrings;
    /// Format string
    std::string formatString;
    /// Separator
    std::string sep;
    /// Lock
    std::mutex lock;

    protected:
    /// Constructor
    CSVBase(std::ostream& out, std::vector<std::string> header, std::string sep = ",");
    /// Destructor
    ~CSVBase() noexcept;

    /// Write line
    void ln(std::format_args args);

    public:
    /// Write header
    void header();
};
//---------------------------------------------------------------------------
/// Typesafe utility for writing csvs
template <typename... Args>
class CSV : public CSVBase {
    public:
    /// Constructor
    CSV(std::ostream& out, std::array<std::string, sizeof...(Args)> header, std::string sep = ",") : CSVBase(out, std::vector(std::make_move_iterator(header.begin()), std::make_move_iterator(header.end())), std::move(sep)) {}
    /// Write line
    void ln(Args... args) {
        return CSVBase::ln(std::make_format_args(args...));
    }
};
//---------------------------------------------------------------------------
}