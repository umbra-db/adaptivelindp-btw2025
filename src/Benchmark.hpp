#pragma once
//---------------------------------------------------------------------------
#include "FunctionRef.hpp"
#include <chrono>
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
/// Utility for benchmarking
class Benchmark {
    /// Measure how much time a function takes to execute
    static std::chrono::duration<double> measureInternal(FunctionRef<void(size_t repetitions)> f);

    public:
    /// Measure how much time a function takes to execute
    template <typename F>
    static std::chrono::duration<double> measure(F&& f) {
        return measureInternal([f{std::forward<F>(f)}](size_t repetitions) {
            for (size_t i = 0; i < repetitions; i++)
                f();
        });
    }
};
//---------------------------------------------------------------------------
}