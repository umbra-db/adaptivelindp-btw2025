#pragma once
//---------------------------------------------------------------------------
#include <cassert>
#include <cstdint>
#include <limits>
#include <vector>
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
// Segment tree based range minimum queries and point updates
class RMQ {
    /// The stored data
    std::vector<int64_t> data;
    /// The original size
    size_t n;

    public:
    /// Constructor
    explicit RMQ(size_t n) : data(n * 2, std::numeric_limits<int64_t>::max()), n(n) {}

    /// Point update, reduce index to value
    constexpr void update(size_t i, int64_t v) {
        i += n;
        assert(v <= data[i]);
        data[i] = v;
        for (; i / 2 >= 1; i /= 2) {
            if (data[i / 2] <= data[i])
                break;
            data[i / 2] = data[i];
        }
    }

    /// Range query, return the minimum value in the range [l, r]
    [[nodiscard]] constexpr int64_t query(size_t l, size_t r) const {
        r += 1;
        int64_t res = std::numeric_limits<int64_t>::max();
        for (l += n, r += n; l < r; l /= 2, r /= 2) {
            if (l & 1)
                res = std::min(res, data[l++]);
            if (r & 1)
                res = std::min(res, data[--r]);
        }
        return res;
    }

    /// Get size
    [[nodiscard]] constexpr size_t size() const { return n; }
};
//---------------------------------------------------------------------------
}
//---------------------------------------------------------------------------