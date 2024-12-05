#pragma once
//---------------------------------------------------------------------------
#include <utility>
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
/// Utilities for lambda functions
class Lambda {
    public:
    /// Helper for recursive invocation
    template <typename F>
    struct RecursiveHelper {
        /// The actual lambda
        F f;
        /// Invoke the lambda
        template <typename... Ts>
        auto operator()(Ts&&... args) const {
            return f(*this, std::forward<Ts>(args)...);
        }
    };
    /// Make a lambda recursive. Can be avoided if compiler supports deducing this.
    template <typename F>
    static auto recursive(F&& f) {
        return RecursiveHelper<F>{std::forward<F>(f)};
    }
};
//---------------------------------------------------------------------------
}