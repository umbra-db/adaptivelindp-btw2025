#pragma once
//---------------------------------------------------------------------------
#include <cstdint>
#include <utility>
#ifdef _MSC_VER
#include <intrin.h>
#pragma intrinsic(_umul128)
#endif
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
/// Int128 utilities
class Int128 {
    public:
    /// 128bit multiply
    static std::pair<uint64_t, uint64_t> mul(uint64_t a, uint64_t b) noexcept {
#ifdef _MSC_VER
        uint64_t high;
        uint64_t low = _umul128(a, b, &high);
        return {low, high};
#else
        auto x = static_cast<unsigned __int128_t>(a) * b;
        return {static_cast<uint64_t>(x), static_cast<uint64_t>(x >> 64)};
#endif
    }
    /// 128bit mix
    static uint64_t mix(uint64_t a, uint64_t b) noexcept {
        auto [low, high] = mul(a, b);
        return low ^ high;
    }
};
//---------------------------------------------------------------------------
}
//---------------------------------------------------------------------------