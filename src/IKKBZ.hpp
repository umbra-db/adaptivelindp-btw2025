#pragma once
//---------------------------------------------------------------------------
#include "FunctionRef.hpp"
#include <span>
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
class QueryGraph;
//---------------------------------------------------------------------------
/// Linearization transfer setting
enum class LinerizationTransfer {
    None,
    Basic,
    Sorted
};
/// Options for IKKBZ
struct IKKBZOptions {
    /// Run with test checks
    bool test = false;
    /// Lineraization transfer
    LinerizationTransfer transfer = LinerizationTransfer::Sorted;
};
//---------------------------------------------------------------------------
/// Implementation of the IK/KBZ algorithm
class IKKBZ {
    public:
    using Options = IKKBZOptions;
    /// Compute the optimal relation order. Invoke the callback with the changed relation order prefix for every head relation.
    static void optimize(const QueryGraph& q, FunctionRef<void(const std::span<const unsigned>&)> callback, Options opt = {});
};
//---------------------------------------------------------------------------
}