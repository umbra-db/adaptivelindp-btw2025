#pragma once
//---------------------------------------------------------------------------
#include <bit>
#include <cstdint>
#include <random>
#include "Int128.hpp"
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
/// A simple random source based on wyrand
class Random {
    /// The seed
    uint64_t seed;
    /// Add constant
    static constexpr uint64_t addConstant = 0x2d358dccaa6c78a5ull;
    /// Mul constant
    static constexpr uint64_t xorConstant = 0x8bb84b93962eacc9ull;

    public:
    /// Constructor
    explicit constexpr Random(uint64_t seed = 0) noexcept : seed(seed) {}
    /// Result type
    using result_type = uint64_t;
    /// Min
    static constexpr result_type min() noexcept { return 0; }
    /// Max
    static constexpr result_type max() noexcept { return ~0ull; }

    /// Step
    uint64_t next() noexcept {
        seed += addConstant;
        return Int128::mix(seed, seed ^ xorConstant);
    }
    /// Step
    uint64_t operator()() noexcept {
        return next();
    }
    /// Generate random number in range
    uint64_t nextRange(uint64_t limit) noexcept {
        std::uniform_int_distribution<uint64_t> dist(0, limit - 1);
        return dist(*this);
    }
    /// Generate random floating point in [0, 1)
    double nextUnit() noexcept {
        std::uniform_real_distribution<double> dist(0.0, 1.0);
        return dist(*this);
    }
};
//---------------------------------------------------------------------------
}
//---------------------------------------------------------------------------