#pragma once
//---------------------------------------------------------------------------
#include <iostream>
#include <tuple>
#include <vector>
#include <source_location>
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
namespace std {
//---------------------------------------------------------------------------
template <typename A, typename B>
std::ostream& operator<<(std::ostream& out, const std::pair<A, B>& p)
// Print pair
{
    return out << "(" << p.first << ", " << p.second << ")";
}
//---------------------------------------------------------------------------
template <typename... Ts>
std::ostream& operator<<(std::ostream& out, const std::tuple<Ts...>& t)
// Print tuple
{
    out << "(";
    std::apply([&](auto&&... args) {
        bool first = true;
        auto pp = [&](const auto& v){
            if (!first) out << ", ";
            else first = false;
            out << v;
        };
        ((pp(args)), ...);
        }, t);
    return out << ")";
}
//---------------------------------------------------------------------------
template <typename T>
std::ostream& operator<<(std::ostream& out, const std::vector<T>& v)
// Print vector
{
    out << "[";
    bool first = true;
    for (const auto& x : v) {
        if (!first) out << ", ";
        else first = false;
        out << x;
    }
    return out << "]";
}
//---------------------------------------------------------------------------
template <typename T>
std::ostream& operator<<(std::ostream& out, std::span<const T> v)
// Print vector
{
    out << "[";
    bool first = true;
    for (const auto& x : v) {
        if (!first) out << ", ";
        else first = false;
        out << x;
    }
    return out << "]";
}
//---------------------------------------------------------------------------
inline std::ostream& operator<<(std::ostream& out, const std::source_location& loc)
// Print source location
{
    return out << loc.file_name() << ':'
               << loc.line() << ':'
               << loc.column() << " `"
               << loc.function_name() << "`";
}
//---------------------------------------------------------------------------
template <typename T1, typename T2>
void assertEq(const T1& a, const T2& b, std::source_location loc = std::source_location::current())
// Check two values for equality
{
    if (a != b) {
        std::cerr << "Error: " << a << " == " << b << " does not hold!\n" << loc << std::endl;
        throw std::runtime_error("err");
    }
}
//---------------------------------------------------------------------------
template <typename T1, typename T2>
void assertEq(const T1& a, const T2& b, auto&& extra, std::source_location loc = std::source_location::current())
// Check two values for equality
{
    if (a != b) {
        std::cerr << "Error: " << a << " == " << b << " does not hold!\n" << loc << std::endl;
        std::cerr << "Info: " << extra << std::endl;
        throw std::runtime_error("err");
    }
}
#ifndef NDEBUG
#define ASSERTEQ(A, B) assertEq((A), (B))
#else
#define ASSERTEQ(A, B) do {} while(false)
#endif
//---------------------------------------------------------------------------
}
//---------------------------------------------------------------------------