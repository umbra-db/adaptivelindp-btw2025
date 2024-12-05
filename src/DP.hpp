#pragma once
//---------------------------------------------------------------------------
#include "FunctionRef.hpp"
#include "QueryGraph.hpp"
#include "Algo.hpp"
#include "MDQ.hpp"
#include <memory>
#include <span>
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
class QueryGraph;
//---------------------------------------------------------------------------
/// Implementation of dp
class DP {
    public:
    /// Dp size linear
    static std::vector<unsigned> optimizeLinear(const QueryGraph& q, unsigned startNode, std::span<const unsigned> expected);
};
//---------------------------------------------------------------------------
/// DP Options
struct DPOptions {
    /// Use the optimal algorithm
    bool newAlgo = true;
};
class IncrementalDP {
    /// The implementation
    struct Impl;
    /// The implementation
    std::unique_ptr<Impl> impl;

    public:
    /// Constructor
    IncrementalDP(const QueryGraph& qg);
    /// Destructor
    ~IncrementalDP() noexcept;

    using Options = DPOptions;

    /// Update the current order of the nodes
    void updateNew(std::span<const unsigned> changed, auto&& callback);
    /// Update the current order of the nodes
    void updateOld(std::span<const unsigned> changed, auto&& callback);
    /// Update the current order of the nodes
    void update(std::span<const unsigned> changed, auto&& callback, Options opt = {}) {
        if (opt.newAlgo) updateNew(changed, callback);
        else updateOld(changed, callback);
    }

    /// The node at index
    unsigned operator[](unsigned index) const;
};
//---------------------------------------------------------------------------
/// The implementation
struct IncrementalDP::Impl {
    using Edge = QueryGraph::Edge;
    /// All edges
    std::vector<Edge> edges;
    /// The ranges of edges for every node
    std::vector<unsigned> ranges;
    /// For every i: every j such that [i, j] is connected
    std::vector<std::vector<unsigned>> conn;
    /// For every j: every i such that [i, j] is connected
    std::vector<std::vector<unsigned>> connR;
    /// For every j: the smallest i such that [i, j] is connected
    MDQ<int32_t> earliest;
    /// Incoming edge
    struct IncEdge {
        int32_t target = std::numeric_limits<int32_t>::max();
        float selectivity = 0.0f;
        constexpr IncEdge() = default;
        constexpr IncEdge(int32_t target, float selectivity) : target(target), selectivity(selectivity) {}
        auto operator<=>(const IncEdge& o) const { return target <=> o.target; }
    };
    /// For every j: the smallest i such that there is an edge i--j
    MDQ<IncEdge> incomingEdge;
    /// The current array of nodes
    std::vector<unsigned> nodes;
    /// The order of the current array of nodes
    std::vector<unsigned> order;
    /// Storage for old dp
    std::vector<char> dp;

    /// Get the outgoing edges
    [[nodiscard]] constexpr std::span<const Edge> getEdges(unsigned node) const {
        auto begin = ranges[node];
        auto end = ranges[node + 1];
        return {edges.data() + begin, end - begin};
    }

    /// Constructor
    Impl(const QueryGraph& qg);
    /// Update the current order of the nodes
    void updateNew(std::span<const unsigned> changed, auto&& callback);
    /// Update the current order of the nodes
    void updateOld(std::span<const unsigned> changed, auto&& callback);
};
//---------------------------------------------------------------------------
inline unsigned IncrementalDP::operator[](unsigned index) const
// The node at index
{
    return impl->nodes[index];
}
//---------------------------------------------------------------------------
void IncrementalDP::Impl::updateNew(std::span<const unsigned> changed, auto&& callback)
// Update the current order of the nodes
{
    if (earliest.size() == 0) {
        assert(incomingEdge.size() == 0);
        earliest = MDQ<int32_t>(changed.size());
        incomingEdge = MDQ<IncEdge>(changed.size());
    }
    auto cs = changed.size();
    for (size_t i = 0; i < cs; i++) {
        nodes[i] = changed[changed.size() - 1 - i];
        order[nodes[i]] = i;
        conn[i].clear();
        connR[i].clear();
        earliest.set(i, i);
        incomingEdge.set(i, IncEdge(i, 0.0f));
    }
    cs = changed.size();
    for (size_t i = cs; i < nodes.size(); i++) {
        if (earliest[i] < changed.size()) {
            // connR is strictly decreasing. Find the first value that is <= changed.size()
            auto newSize = lower_bound_fast(connR[i].begin(), connR[i].end(), changed.size(), std::greater<unsigned>{}) - connR[i].begin();
            if (newSize < connR[i].size() && connR[i][newSize] == changed.size())
                newSize++;
            connR[i].resize(newSize);
            assert(!connR[i].empty());
            earliest.set(i, connR[i].back());
            assert(earliest[i] >= changed.size());
        }
        if (incomingEdge[i].target < changed.size()) {
            // TODO: should we optimize the following to log(n)?
            //  We could constantly reorder incoming edges of individual nodes while iterating over the graph
            incomingEdge.set(i, IncEdge(i, 0.0f));
            for (auto& t : getEdges(nodes[i]))
                if (order[t.target] < incomingEdge[i].target && order[t.target] >= changed.size())
                    incomingEdge.update(i, IncEdge(order[t.target], t.selectivity));
        }
    }

    for (unsigned i = changed.size() - 1; i != ~0u; --i) {
        for (auto& j : getEdges(nodes[i]))
            if (order[j.target] > i)
                incomingEdge.update(order[j.target], IncEdge(i, j.selectivity));

        size_t last = i;
        do {
            earliest.update(last, i);
            conn[i].emplace_back(last);
            connR[last].emplace_back(i);

            if (last + 1 >= nodes.size())
                break;

            // The first index with which we are connected with over an edge
            auto nextInd = incomingEdge.query(last + 1, IncEdge(last, 0.0f));
            if (nextInd >= nodes.size())
                break;

            float sel = incomingEdge[nextInd].selectivity;
            // Iterate over connected components
            for (auto it = lower_bound_fast(conn[last + 1].begin(), conn[last + 1].end(), nextInd); it != conn[last + 1].end(); ++it)
                callback(i, last, *it, sel);

            // TODO: should we remove the following and rely on connected component iteration?
            last = earliest.query(nextInd, last + 1);
        } while (last < nodes.size());
    }
}
//---------------------------------------------------------------------------
void IncrementalDP::Impl::updateOld(std::span<const unsigned> changed, auto&& callback)
// Update the current order of the nodes
{
    bool init = dp.empty();
    if (init) {
        dp.resize(nodes.size() * nodes.size());
        assert(changed.size() == nodes.size());
    }
    for (unsigned i = 0; i < changed.size(); i++) {
        nodes[i] = changed[changed.size() - 1 - i];
        order[nodes[i]] = i;
        if (!init) {
            __builtin_memset(dp.data() + i * nodes.size(), 0, nodes.size() - i);
        }
        dp[i * nodes.size()] = 1;
    }
    auto* idp = dp.data() + (changed.size() - 1) * nodes.size();
    for (unsigned i = changed.size() - 1; i != ~0u; --i, idp -= nodes.size()) {
        auto* ikdpx = idp;
        for (unsigned k = i; k < nodes.size() - 1; k++, ikdpx++) {
            if (*ikdpx != 1)
                continue;

            auto* ijdpx = ikdpx + 1;
            auto* kjdp = dp.data() + (k + 1) * nodes.size();
            auto* kjdpx = kjdp;
            for (unsigned j = k + 1; j < nodes.size(); j++, ijdpx++, kjdpx++) {
                if (*kjdpx != 1)
                    continue;

                float sel = 1.0f;
                auto hasEdge = [&]() {
                    // TODO: implement with binary search
                    for (unsigned a = i; a <= k; a++) {
                        for (auto& b : getEdges(nodes[a])) {
                            if (order[b.target] > k && order[b.target] <= j) {
                                sel = b.selectivity;
                                return true;
                            }
                        }
                    }
                    return false;
                };
                // TODO: Explain assumption
                //  conn(i, j) and conn(i, k) and conn(k + 1, j)
                //  --> there is an edge between [i, k] and [k + 1, j]
                if (!*ijdpx) {
                    if (hasEdge()) *ijdpx = 1;
                    else *ijdpx = 2;
                }
                assert(*ijdpx == (hasEdge() ? 1 : 2));
                if (*ijdpx != 1)
                    continue;
                callback(i, k, j, sel);
            }
        }
    }
}
//---------------------------------------------------------------------------
void IncrementalDP::updateNew(std::span<const unsigned> changed, auto&& callback)
// Update the current order of the nodes
{
    return impl->updateNew(changed, callback);
}
//---------------------------------------------------------------------------
void IncrementalDP::updateOld(std::span<const unsigned> changed, auto&& callback)
// Update the current order of the nodes
{
    return impl->updateOld(changed, callback);
}
//---------------------------------------------------------------------------
}