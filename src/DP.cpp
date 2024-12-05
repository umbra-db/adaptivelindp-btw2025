#include "Algo.hpp"
#include "DP.hpp"
#include "MDQ.hpp"
#include "QueryGraph.hpp"
#include <cassert>
#include <expected>
#include <queue>
#include <unordered_map>
#include <set>
#include <vector>
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
using namespace std;
//---------------------------------------------------------------------------
namespace fastoptim {
//---------------------------------------------------------------------------
IncrementalDP::Impl::Impl(const QueryGraph& qg)
    : conn(qg.size()), connR(qg.size()), earliest(0), incomingEdge(0), nodes(qg.size()), order(qg.size())
// Constructor
{
    vector<vector<Edge>> incoming(qg.size()), outgoing(qg.size());
    for (unsigned i = 0; i < qg.size(); i++) {
        for (auto& e : qg.getEdges(i)) {
            incoming[e.target].push_back(Edge{i, e.selectivity});
            outgoing[i].push_back(Edge{e.target, e.selectivity});
        }
    }
    edges.reserve(qg.edges.size() * 2);
    ranges.reserve(qg.size() + 1);
    ranges.push_back(0);
    for (size_t i = 0; i < qg.size(); i++) {
        edges.insert(edges.end(), incoming[i].begin(), incoming[i].end());
        edges.insert(edges.end(), outgoing[i].begin(), outgoing[i].end());
        ranges.push_back(edges.size());
    }
}
//---------------------------------------------------------------------------
/// Constructor
IncrementalDP::IncrementalDP(const QueryGraph& qg) : impl(make_unique<Impl>(qg)) {  }
/// Destructor
IncrementalDP::~IncrementalDP() noexcept = default;
//---------------------------------------------------------------------------
vector<unsigned> DP::optimizeLinear(const QueryGraph& q, unsigned startNode, span<const unsigned> expected)
// Dp size linear
{
    assert(q.size() < 64);
    struct Plan {
        float cardinality = numeric_limits<float>::infinity();
        float cost = numeric_limits<float>::infinity();
        unsigned rel = 0;
    };
    vector<unordered_map<uint64_t, Plan>> plans(q.size());
    vector<vector<pair<unsigned, float>>> adj(q.size());
    for (unsigned i = 0; i < q.size(); i++) {
        for (auto& e : q.getEdges(i)) {
            adj[i].emplace_back(e.target, e.selectivity);
            adj[e.target].emplace_back(i, e.selectivity);
        }
    }

    plans[0].emplace(1ull << startNode, Plan{q.nodes[startNode].cardinality, 0.0f, startNode});
    for (size_t iter = 0; iter < q.size() - 1; iter++) {
        auto& plans1 = plans[iter];
        auto& plans2 = plans[iter + 1];
        for (auto& [k, v] : plans1) {
            if (v.cardinality == numeric_limits<float>::infinity())
                continue;
            // Find connected
            for (size_t c = 0; c < q.size(); c++) {
                if (k & (1ull << c))
                    continue;

                auto it = plans2.find(k | (1ull << c));
                if (it == plans2.end()) {
                    bool foundEdge = false;
                    float mult = q.nodes[c].cardinality;
                    for (auto& [target, selectivity] : adj[c]) {
                        if (k & (1ull << target)) {
                            foundEdge = true;
                            mult *= selectivity;
                        }
                    }
                    it = plans2.emplace_hint(it, k | (1ull << c), Plan{});
                    if (foundEdge)
                        it->second.cardinality = v.cardinality * mult;
                }
                if (it->second.cardinality < numeric_limits<float>::infinity()) {
                    float cost = v.cost + it->second.cardinality;
                    if (cost < it->second.cost) {
                        it->second.cost = cost;
                        it->second.rel = c;
                    }
                }
            }
        }
    }
    // Reconstruct path
    vector<unsigned> rels;
    uint64_t full = (1ull << q.size()) - 1;
    uint64_t cur = full;
    unsigned n = q.size() - 1;
    while (cur) {
        auto it = plans[n].find(cur);
        assert(it != plans[n].end());
        rels.push_back(it->second.rel);
        cur -= 1ull << it->second.rel;
        n--;
    }
    assert(rels.size() == expected.size());
    if (!equal(rels.begin(), rels.end(), expected.begin())) {
        double cost = 0.0;
        uint64_t k = 1ull << rels.back();
        unsigned n = 1;
        for (unsigned i = rels.size() - 2; i != ~0u; i--) {
            k |= 1ull << rels[i];
            cost += plans[n].at(k).cardinality;
            n++;
        }
        double ikkbzcost = cost;
        double dpcost = plans.back().at(full).cost;
        if (abs(ikkbzcost - dpcost) / dpcost > 1e-5) {
            cerr << "IKKBZ cost: " << cost << " DP cost: " << plans.back().at(full).cost << endl;
            throw runtime_error("error");
        }
    }

    return rels;
}
//---------------------------------------------------------------------------
}
