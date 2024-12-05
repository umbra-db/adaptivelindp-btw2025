#include "QueryGraph.hpp"
#include "Random.hpp"
#include "UnionFind.hpp"
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <array>
#include <cassert>
#include <numeric>
#include <bit>
#include <sstream>
#include <random>
#include <set>
#include <unordered_set>
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
static std::pair<uint64_t, std::string_view> readInt(std::string_view s)
// Read an integer in graph6 format
{
    if (s.empty())
        return {0, {}};
    if (s[0] == 126) {
        if (s.size() >= 2 && s[1] == 126) {
            uint64_t v = 0;
            auto l = min(sizeof(v), s.size());
            memcpy(&v, s.data(), l);
            v = byteswap(v);
            v &= (1ull << 48) - 1;
            // Subtract 63 from all
            v -= 0x3f3f3f3f3f3full;
            return {v, s.substr(l)};
        } else {
            uint32_t v = 0;
            auto l = min(sizeof(v), s.size());
            memcpy(&v, s.data(), l);
            v = byteswap(v);
            v &= (1ull << 24) - 1;
            v -= 0x3f3f3full;
            return {v, s.substr(l)};
        }
    } else {
        return {s[0] - 63, s.substr(1)};
    }
}
//---------------------------------------------------------------------------
[[maybe_unused]] static std::string writeInt(uint64_t v)
// Write an integer in graph6 format
{
    if (v <= 62)
        return string(1, v + 63);
    string result;
    result += 126;
    size_t lim;
    if (v <= 258047) {
        lim = 18;
    } else {
        lim = 36;
        result += 126;
    }
    for (size_t i = lim; lim > 0; lim -= 6) {
        auto st = lim - 6;
        result += ((v >> st) & 0x3f) + 63;
    }
    return result;
}
//---------------------------------------------------------------------------
QueryGraph QueryGraph::fromGraph6(std::string_view s)
// Read graph encoded in Graph6 format. Set cardinalities and selectivities to 1
{
    uint64_t n;
    tie(n, s) = readInt(s);
    if (n == 0)
        return {};
    vector<vector<unsigned>> adj;
    adj.resize(n);

    std::vector<uint8_t> data;
    data.reserve((n * (n - 1) / 2 + 7) / 8);
    auto read8 = [&]() {
        std::array<uint8_t, 8> v{};
        auto l = std::min(sizeof(v), s.size());
        memcpy(&v, s.data(), l);
        s = s.substr(l);
        return std::pair{v, l};
    };
    while (!s.empty()) {
        auto [v, l] = read8();
        uint64_t r = 0;
        for (int i = 0; i < l; i++) {
            auto val = v[i] - 63;
            // swap bits [0..5]
            val = ((val & 0b101010) >> 1) | ((val & 0b010101) << 1);
            val = ((val & 0b001100)) | ((val & 0b000011) << 4) | ((val & 0b110000) >> 4);
            r = r | (val << (6 * i));
        }
        std::array<uint8_t, 6> v2{};
        memcpy(&v2, &r, sizeof(v2));
        data.insert(data.end(), v2.begin(), v2.end());
    }

    int bit = 0;
    for (unsigned i = 0; i < n; i++) {
        for (unsigned j = 0; j < i; j++) {
            if (data[bit / 8] & (1 << (bit % 8))) {
                adj[j].push_back(i);
            }
            bit++;
        }
    }
    return fromAdjList(adj);
}
//---------------------------------------------------------------------------
QueryGraph QueryGraph::fromEdges(std::string_view s)
// Read graph encoded in edges format. Set cardinalities and selectivities to 1
{
    std::istringstream ss(std::string{s});
    vector<vector<unsigned>> adj;
    size_t a, b;
    while (ss >> a >> b) {
        adj.resize(std::max(adj.size(), std::max(a, b) + 1));
        if (a > b)
            std::swap(a, b);
        adj[a].push_back(b);
    }
    return fromAdjList(adj);
}
//---------------------------------------------------------------------------
QueryGraph QueryGraph::fromAdjList(span<const vector<unsigned>> adjList, std::span<const float> relations)
// Build graph from adjacency list
{
    vector<vector<pair<unsigned, float>>> adj;
    adj.reserve(adjList.size());
    for (auto& a : adjList) {
        vector<pair<unsigned, float>> temp;
        temp.reserve(a.size());
        for (auto& b : a)
            temp.push_back({b, 1.0});
        adj.push_back(move(temp));
    }
    return fromAdjList(adj, relations);
}
//---------------------------------------------------------------------------
QueryGraph QueryGraph::fromAdjList(span<const vector<pair<unsigned, float>>> adjList, span<const float> relations)
// Build graph from adjacency list. Set cardinalities and selectivities to 1
{
    // Count total number of edges
    unsigned e = 0;
    for (auto& a : adjList)
        e += a.size();

    QueryGraph result;
    result.nodes.resize(adjList.size());
    for (size_t i = 0; i < relations.size(); i++)
        result.nodes[i].cardinality = relations[i];
    result.edges.resize(e);
    result.ranges.resize(adjList.size() + 1);

    vector<std::pair<unsigned, float>> temp;
    // Fill in the edges
    unsigned pos = 0;
    for (unsigned i = 0; i < adjList.size(); i++) {
        result.ranges[i] = pos;
        temp.resize(0);
        temp.insert(temp.end(), adjList[i].begin(), adjList[i].end());
        sort(temp.begin(), temp.end());
        for (auto& j : temp) {
            result.edges[pos].target = j.first;
            result.edges[pos].selectivity = j.second;
            pos++;
        }
    }
    result.ranges[adjList.size()] = pos;
    assert(pos == e);
    return result;
}
//---------------------------------------------------------------------------
QueryGraph QueryGraph::fromLists(span<const tuple<unsigned, unsigned, float>> edges, span<const float> relations)
// Build graph from edge and relation list
{
    vector<vector<pair<unsigned, float>>> adj;
    for (auto& e : edges) {
        unsigned from, to;
        float selectivity;
        tie(from, to, selectivity) = e;
        if (from > to)
            swap(from, to);
        adj.resize(std::max<unsigned>(adj.size(), to + 1));
        adj[from].push_back({to, selectivity});
    }
    return fromAdjList(adj, relations);
}
//---------------------------------------------------------------------------
QueryGraph QueryGraph::computeMST() const
// Compute the minimum spanning tree of the graph
{
    struct E {
        float selectivity;
        unsigned from, to;
        constexpr auto operator<=>(const E&) const = default;
    };
    // Collect edges
    vector<E> es;
    es.reserve(edges.size());
    for (unsigned i = 0; i < nodes.size(); i++)
        for (auto& e : getEdges(i))
            es.push_back({e.selectivity, i, e.target});
    // Sort edges by selectivity
    sort(es.begin(), es.end());
    // Compute MST
    UnionFind uf(nodes.size());
    // Filter vector
    auto it = remove_if(es.begin(), es.end(), [&](auto& e) {
        if (uf.find(e.from) == uf.find(e.to))
            return true;
        uf.merge(e.from, e.to);
        return false;
    });
    es.erase(it, es.end());
    sort(es.begin(), es.end(), [&](const E& a, const E& b) {
        return tie(a.from, a.to) < tie(b.from, b.to);
    });
    // Build new graph
    QueryGraph result;
    result.nodes = nodes;
    result.edges.reserve(es.size());
    result.ranges.reserve(nodes.size() + 1);
    unsigned lastNode = ~0u;
    for (auto& e : es) {
        if (e.from != lastNode) {
            while (result.ranges.size() <= e.from)
                result.ranges.push_back(result.edges.size());
            lastNode = e.from;
        }
        result.edges.push_back({e.to, e.selectivity});
    }
    while (result.ranges.size() <= result.nodes.size())
        result.ranges.push_back(result.edges.size());
    return result;
}
//---------------------------------------------------------------------------
QueryGraph QueryGraph::computeReverse() const
// Compute the reverse of the graph
{
    vector<vector<pair<unsigned, float>>> adj(nodes.size());
    for (unsigned i = 0; i < nodes.size(); i++)
        for (auto& e : getEdges(i))
            adj[e.target].emplace_back(i, e.selectivity);
    return fromAdjList(adj);
}
//---------------------------------------------------------------------------
void QueryGraph::randomizeWeights(Random& eng)
// Randomize weights
{
    /*set<float> generated;
    generated.insert(-0.1f);
    generated.insert(1.1f);
    auto distance = [&](float v) {
        auto it = generated.lower_bound(v);
        auto v1 = *it;
        ++it;
        auto v2 = *it;
        return min(abs(v1 - v), abs(v2 - v));
    };
    uniform_real_distribution<float> dist(0.0, 1.0);
    for (auto& e : edges) {
        e.selectivity = dist(eng);
        // Make sure we generate values that aren't too close to existing values
        if (edges.size() < 1000) {
            while (distance(e.selectivity) < 1e-5)
                e.selectivity = dist(eng);
        }
        generated.insert(e.selectivity);
    }*/
    uniform_real_distribution<float> dist(0.0, 1.0);
    for (auto& e : edges)
        e.selectivity = dist(eng);
    //for (auto& n : nodes)
    //    n.cardinality = eng.nextUnit() * 1024;
}
//---------------------------------------------------------------------------
QueryGraph QueryGraph::randomTree(unsigned n, double chainRatio, double density, Random& eng)
// Generate a random tree. Set cardinalities and selectivities to 1
{
    if (n == 0)
        return {};
    vector<vector<unsigned>> adj(n);
    vector<unsigned> from;
    vector<unsigned> to;
    from.reserve(n);
    to.reserve(n);
    for (unsigned i = 0; i < n; i++) to.push_back(i);

    auto getTo = [&]() {
        unsigned i = eng.nextRange(to.size());
        auto v = to[i];
        from.push_back(v);
        swap(to[i], to.back());
        to.pop_back();
        return v;
    };

    auto addEdge = [&](unsigned f, unsigned t) {
        if (f > t)
            swap(f, t);
        adj[f].push_back(t);
    };

    getTo();
    size_t chainTarget = min<size_t>(n * chainRatio, n);
    while (from.size() < chainTarget) {
        auto f = from.back();
        auto t = getTo();
        addEdge(f, t);
    }

    while (!to.empty()) {
        auto f = from[eng.nextRange(from.size())];
        auto t = getTo();
        addEdge(f, t);
    }

    if ((n < 3) || (density == 0.0))
        return fromAdjList(adj);

    vector<unordered_set<unsigned>> edgeLookup;
    edgeLookup.reserve(n);
    for (unsigned i = 0; i < n; i++)
        edgeLookup.emplace_back(adj[i].begin(), adj[i].end());

    size_t maxExtra = (n * (n - 1)) / 2 - (n - 1);
    size_t target = ceil(maxExtra * density);
    size_t e = 0;

    // Add additional edges, now that everything is connected
    for (; e < min(target, maxExtra / 2); e++) {
        while (true) {
            auto f = eng.nextRange(n);
            auto t = eng.nextRange(n);
            if (f == t)
                continue;
            if (f > t)
                swap(f, t);
            if (edgeLookup[f].contains(t))
                continue;
            adj[f].push_back(t);
            edgeLookup[f].insert(t);
            break;
        }
    }
    if (e < target) {
        // Handle dense case
        vector<pair<unsigned, unsigned>> remainingEdges;
        for (unsigned i = 0; i < n; i++)
            for (unsigned j = i + 1; j < n; j++)
                if (!edgeLookup[i].contains(j))
                    remainingEdges.emplace_back(i, j);
        edgeLookup.clear();

        for (; e < target; e++) {
            auto ind = eng.nextRange(remainingEdges.size());
            auto [f, t] = remainingEdges[ind];
            adj[f].push_back(t);
            remainingEdges[ind] = remainingEdges.back();
            remainingEdges.pop_back();
        }
    }


    return fromAdjList(adj);
}
//---------------------------------------------------------------------------
ostream& operator<<(ostream& out, const QueryGraph& q)
// Write graph out in dot format
{
    // Graph is undirected
    // Annotate nodes with cardinality and edges with selectivity if the value is not 1
    out << "graph G {";
    for (unsigned i = 0; i < q.nodes.size(); i++) {
        if (q.nodes[i].cardinality != 1)
            out << i << "[label=\"" << i << "(" << q.nodes[i].cardinality << ")\"];";
    }
    for (unsigned i = 0; i < q.nodes.size(); i++) {
        for (auto& e : q.getEdges(i)) {
            if (e.selectivity != 1)
                out << i << "--" << e.target << "[label=\"" << e.selectivity << "\"];";
            else
                out << i << "--" << e.target << ";";
        }
    }
    out << "}";
    return out;
}
//---------------------------------------------------------------------------
}
//---------------------------------------------------------------------------