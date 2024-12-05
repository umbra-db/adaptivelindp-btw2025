#pragma once
//---------------------------------------------------------------------------
#include "FunctionRef.hpp"
#include "DP.hpp"
#include "Plan.hpp"
#include "IKKBZ.hpp"
#include "PrintUtil.hpp"
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
struct Plan;
//---------------------------------------------------------------------------
/// Options for LinDP
struct LinDPOptions {
    /// IKKBZ options
    IKKBZ::Options ikkbz{};
    /// DP options
    IncrementalDP::Options dp{};
};
//---------------------------------------------------------------------------
/// Implementation of the LinDP algorithm
class LinDP {
    public:
    using Options = LinDPOptions;
    /// Compute the optimal plan
    static Plan* optimize(const QueryGraph& q, std::span<Plan* const> basePlans, auto&& combine, Options opt = {});
};
//---------------------------------------------------------------------------
Plan* LinDP::optimize(const QueryGraph& q, std::span<Plan* const> basePlans, auto&& combine, Options opt)
// Compute the optimal plan
{
    struct Entry {
        private:
        uint64_t value = 0;

        public:
        Plan* getLast() const { return reinterpret_cast<Plan*>(value >> 16); }
        unsigned getVersion() const { return static_cast<uint16_t>(value); }

        Plan* get(unsigned version) {
            if (getVersion() != version) return nullptr;
            return getLast();
        }
        void set(Plan* ptr, unsigned version) {
            value = reinterpret_cast<uint64_t>(ptr) << 16 | static_cast<uint16_t>(version);
        }
    };
    std::vector<Entry> best(q.size() * q.size());

    assert(q.size() < std::numeric_limits<uint16_t>::max());

    Plan* veryBest = nullptr;

    unsigned version = 0;
    IncrementalDP dp(q);
    IKKBZ::optimize(q, [&](std::span<const unsigned> changed) {
        for (size_t i = 0; i < changed.size(); i++)
            best[i * q.size()].set(basePlans[changed[changed.size() - i - 1]], version);

        auto callback = [&](unsigned i, unsigned k, unsigned j, float sel) {
            auto* base = best.data() + i * q.size() - i;
            Plan* target = base[j].get(version);
            Plan* left = base[k].getLast();
            Plan* right = best[(k + 1) * q.size() + j - (k + 1)].getLast();
            target = combine(target, left, right, sel);
            base[j].set(target, version);
        };
        dp.update(changed, callback, opt.dp);

        auto& b = best[0 * q.size() + q.size() - 1];
        if (b.get(version) != nullptr) {
            if (veryBest == nullptr || b.getLast()->cost < veryBest->cost)
                veryBest = best[0 * q.size() + q.size() - 1].getLast();
        }

        version++;
    }, opt.ikkbz);

    return veryBest;
}
//---------------------------------------------------------------------------
}