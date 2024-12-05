#pragma once
//---------------------------------------------------------------------------
#include "RMQ.hpp"
#include <algorithm>
#include <bit>
#include <iosfwd>
#include <iostream>
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
/// Multi-dimensional range query structure
struct MDQOld {
    private:
    /// The underlying RMQ
    RMQ f;

    public:
    /// Constructor
    explicit MDQOld(size_t n) : f(n) {}

    // Set the value at index i to v
    constexpr void update(size_t i, int64_t v) {
        f.update(i, v);
    }

    // Get the minimum index j >= i whose value is <= v
    [[nodiscard]] constexpr size_t query(size_t i, int64_t v) const {
        // Find first j in [i, n) such that queryInternal(i, j) <= v using binary search
        // queryInternal(i, j) is monotonically decreasing over j
        size_t l = i, r = f.size() - 1;
        if (f.query(i, r) > v)
            return f.size();
        while (l < r) {
            size_t m = l + (r - l) / 2;
            auto x = f.query(i, m);
            if (x <= v)
                r = m;
            else
                l = m + 1;
        }
        return l;
    }
};
//---------------------------------------------------------------------------
/// Multi-dimensional range query structure
/// Based on segment tree from https://codeforces.com/blog/entry/18051
template <typename T>
struct MDQ {
    private:
    /// The original size
    size_t orgN;
    /// The size
    size_t n;
    /// The stored data
    std::vector<T> data;

    template <typename T2>
    friend std::ostream& operator<<(std::ostream& out, const MDQ<T2>& mdq);

    public:
    /// Constructor
    explicit MDQ(size_t orgN) : orgN(orgN), n(std::bit_ceil(orgN)), data(this->n * 2, std::numeric_limits<T>::max()) {}

    /// Size
    [[nodiscard]] constexpr size_t size() const { return orgN; }

    /// Reduce the value at index i to v
    constexpr void update(size_t i, T v) {
        i += n;
        assert(v <= data[i]);
        data[i] = v;
        for (; i / 2 >= 1; i /= 2) {
            if (data[i / 2] <= data[i])
                break;
            data[i / 2] = data[i];
        }
    }
    /// Set the value at index i to v
    constexpr void set(size_t i, T v) {
        i += n;
        data[i] = v;
        for (i /= 2; i >= 1; i /= 2)
            data[i] = std::min(data[i * 2], data[i * 2 + 1]);
    }
    /// Get the current value
    [[nodiscard]] constexpr T operator[](size_t i) const {
        assert(i < n);
        return data[i + n];
    }

    /// Get the minimum index j >= i whose value is <= v
    [[nodiscard]] constexpr size_t queryInternal(size_t i, T v) const {
        // Find first j in [i, n) such that queryInternal(i, j) <= v using binary search
        auto l = i + n;

        auto al = i;
        auto as = 1;

        // Go upwards, find first range that contains a value <= v
        while (true) {
            if (data[l] <= v) break;
            // No range could be found that contains a value <= v
            if (al + as >= n) return orgN;
            bool odd = l & 1;
            al += odd * as;
            l += odd;
            l /= 2;
            as *= 2;
        }
        while (l * 2 < data.size()) {
            l *= 2;
            l += data[l] > v;
        }
        assert(l - n >= i);
        assert((l - n) == std::find_if(data.begin() + n + i, data.end(), [v](auto x) { return x <= v; }) - data.begin() - n);
        return l - n;
    }

    /// Get the minimum index j >= i whose value is <= v
    [[nodiscard]] constexpr size_t query(size_t i, T v) const {
        //static constexpr size_t fastCount = 8;
        //if (orgN - i <= fastCount) {
        //    for (unsigned j = i; j < orgN; j++)
        //        if (data[n + j] <= v)
        //            return j;
        //    return orgN;
        //}
        //for (unsigned j = i; j < i + fastCount; j++)
        //    if (data[n + j] <= v)
        //        return j;
        //return queryInternal(i + fastCount, v);
        return queryInternal(i, v);
    }
};
//---------------------------------------------------------------------------
template <typename T>
std::ostream& operator<<(std::ostream& out, const MDQ<T>& mdq);
//---------------------------------------------------------------------------
}