#include "IKKBZ.hpp"
#include "DP.hpp"
#include "Lambda.hpp"
#include "SmallVector.hpp"
#include "QueryGraph.hpp"
#include "PrintUtil.hpp"
#include <algorithm>
#include <cassert>
#include <limits>
#include <ranges>
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
using namespace std;
//---------------------------------------------------------------------------
namespace fastoptim {
//---------------------------------------------------------------------------
void IKKBZ::optimize(const QueryGraph& q, FunctionRef<void(const span<const unsigned>&)> callback, Options opt)
// Compute the optimal relation order. Invoke the callback with the changed relation order prefix for every head relation.
{
    // Get the mst
    auto mst = q.computeMST();
    auto mstr = mst.computeReverse();

    struct Edge {
        unsigned parent = ~0u;
        float selectivity = -1;
    };
    // Compute the tree structure
    vector<Edge> parents(q.size());
    vector<vector<unsigned>> adj(q.size());
    for (unsigned i = 0; i < q.size(); i++) {
        for (auto& e : mstr.getEdges(i))
            adj[i].push_back(e.target);
        for (auto& e : mst.getEdges(i))
            adj[i].push_back(e.target);
    }

    unsigned startNode = 0;
    /*{
        // The following does not help
        // We want to start from one of the corners of the longest path
        auto findFarthest = [&](unsigned start) {
            struct Entry {
                unsigned cur;
                unsigned parent;
                unsigned distance;
            };
            // Use dfs to find the farthest node to 0
            vector<Entry> ms;
            ms.emplace_back(start, start, 0);
            pair<unsigned, unsigned> farthest = {0, start};
            while (!ms.empty()) {
                auto [cur, parent, distance] = ms.back();
                ms.pop_back();
                farthest = max(farthest, {distance, cur});
                for (auto& child : adj[cur])
                    if (child != parent)
                        ms.emplace_back(child, cur, distance + 1);
            }
            return farthest.second;
        };
        startNode = findFarthest(startNode);
    }*/

    {
        struct Entry {
            unsigned cur;
            unsigned parent;
        };
        vector<Entry> ms;
        ms.emplace_back(startNode, startNode);
        auto iterEdges = [&](unsigned cur, auto&& f) {
            for (auto& edge : mstr.getEdges(cur)) f(edge);
            for (auto& edge : mst.getEdges(cur)) f(edge);
        };
        while (!ms.empty()) {
            auto e = ms.back();
            ms.pop_back();
            iterEdges(e.cur, [&](auto& edge) {
                if (edge.target == e.parent) return;
                parents[edge.target] = {e.cur, edge.selectivity * q.nodes[e.cur].cardinality};
                ms.emplace_back(edge.target, e.cur);
            });
        }
    }
    for (auto& v : adj) sort(v.begin(), v.end());


    struct Node {
        float cost = 0.0;
        float tree = 0.0;
        unsigned rel = 0;
        constexpr auto operator<=>(const Node& other) const noexcept { return (tree - 1.0) * other.cost <=> (other.tree - 1.0) * cost; }
    };
    auto computeNode = [&](unsigned cur) -> Node {
        float t = parents[cur].selectivity * q.nodes[cur].cardinality;
        Node node{t, t, cur};
        return node;
    };
    vector<Node> nodeCache;
    nodeCache.reserve(q.size());
    for (unsigned i = 0; i < q.size(); i++)
        nodeCache.push_back(computeNode(i));

    struct Link {
        unsigned prev, next;
    };

    SmallVector<Node> temp;
    auto mergeChains = [&](span<SmallVector<Node>> chains) -> SmallVector<Node> {
        for (size_t i = 0; i < chains.size(); i++) {
            if (chains[i].empty()) {
                swap(chains[i], chains.back());
                chains = chains.subspan(0, chains.size() - 1);
                i--;
            }
        }
        if (chains.empty()) return {};
        if (chains.size() == 1) return move(chains[0]);
        // Reuse existing memory
        SmallVector<Node> result = move(temp);
        result.clear();
        struct Entry {
            float cost;
            float tree;
            unsigned index;
            constexpr auto operator<=>(const Entry& other) const noexcept { return (tree - 1.0) * other.cost <=> (other.tree - 1.0) * cost; }
        };
        SmallVector<Entry> entries;
        for (unsigned i = 0; i < chains.size(); i++)
            entries.push_back({chains[i].back().cost, chains[i].back().tree, i});
        // Use a heap for efficient merging of an arbitrary number of chains
        make_heap(entries.begin(), entries.end(), greater<>{});
        while (entries.size() > 2) {
            auto e = entries.front();
            auto& chain = chains[e.index];
            if (chain.size() == 1) {
                pop_heap(entries.begin(), entries.end(), greater<>{});
                entries.pop_back();
            } else {
                entries.front() = {chain[chain.size() - 2].cost, chain[chain.size() - 2].tree, e.index};
                // Sift down
                unsigned i = 0;
                while (true) {
                    unsigned j = 2 * i + 1;
                    if (j >= entries.size()) break;
                    if (j + 1 < entries.size() && entries[j + 1] < entries[j]) j++;
                    if (entries[i] < entries[j]) break;
                    swap(entries[i], entries[j]);
                    i = j;
                }
            }
            result.push_back(chain.back());
            chain.pop_back();
        }
        // Two-pointers merge if there are only two chains left
        auto& chain1 = chains[entries[0].index];
        auto& chain2 = chains[entries[1].index];
        while (!chain1.empty() && !chain2.empty()) {
            if (chain1.back() < chain2.back()) {
                result.push_back(chain1.back());
                chain1.pop_back();
            } else {
                result.push_back(chain2.back());
                chain2.pop_back();
            }
        }
        if (!chain1.empty()) {
            reverse(chain1.begin(), chain1.end());
            result.reserve(result.size() + chain1.size());
            for (auto& n : chain1) result.push_back(n);
        } else if (!chain2.empty()) {
            reverse(chain2.begin(), chain2.end());
            result.reserve(result.size() + chain2.size());
            for (auto& n : chain2) result.push_back(n);
        }
        reverse(result.begin(), result.end());
        // Try to minimize memory allocations by saving the vector with the largest capacity
        for (auto& c : chains)
            if (c.capacity() > temp.capacity())
                temp = move(c);
        return result;
    };
    auto addToChain = [&](span<Link> links, SmallVector<Node>& result, Node root) -> void {
        while (!result.empty() && (root > result.back())) {
            // Merge and link
            auto r = root.rel;
            auto n = result.back().rel;
            // Merge the two cycles
            // r -> ... -> re -> r and n -> ... -> ne -> n
            // should become
            // r -> ... -> re -> n -> ... -> ne -> r
            auto re = links[r].prev;
            auto ne = links[n].prev;
            links[re].next = n;
            links[n].prev = re;
            links[ne].next = r;
            links[r].prev = ne;

            root.cost += root.tree * result.back().cost;
            root.tree *= result.back().tree;
            //root.rank = (root.tree - 1) / root.cost;
            result.pop_back();
        }
        result.push_back(root);
    };
    auto buildChain = Lambda::recursive([&](auto& self, span<Link> links, unsigned cur, unsigned parent = ~0u) -> SmallVector<Node> {
        SmallVector<SmallVector<Node>> chains;
        for (unsigned child : adj[cur])
            if (child != parent)
                chains.push_back(self(links, child, cur));
        SmallVector<Node> result = mergeChains(chains);
        if (parent != ~0u) {
            // Normalize with the root
            addToChain(links, result, nodeCache[cur]);
        } else {
            result.push_back(nodeCache[cur]);
        }
        return result;
    });
    auto denormalize = [&](span<Link> links, const SmallVector<Node>& normalized, span<unsigned> result) -> unsigned {
        auto d = result.data();
        unsigned ind = result.size() - 1;
        unsigned unmod = 0;
        for (auto node : ranges::reverse_view(normalized)) {
            unsigned r = node.rel;
            do {
                if (d[ind] == r) unmod++;
                else unmod = 0;
                d[ind--] = r;
                auto orgR = r;
                r = links[orgR].next;
                links[orgR].next = links[orgR].prev = orgR;
            } while (r != node.rel);
        }
        assert(ind == ~0u);
        return result.size() - unmod;
    };
    vector<char> group(q.size());
    vector<unsigned> working0, working1;
    array<SmallVector<Node>, 2> ws{};
    auto rotate = [&](span<Link> links, span<unsigned> rels, unsigned curRoot, unsigned nextRoot) -> unsigned {
        if (rels.size() <= 1)
            return 0;
        parents[curRoot].parent = nextRoot;
        parents[curRoot].selectivity = parents[nextRoot].selectivity;
        nodeCache[nextRoot] = Node{0.0, numeric_limits<float>::min(), nextRoot};
        nodeCache[curRoot] = computeNode(curRoot);

        group[curRoot] = 0;
        group[nextRoot] = 1;

        assert(rels.back() == curRoot);
        working0.clear();
        working1.clear();
        working0.push_back(curRoot);
        // Iterate until we reach the next root
        unsigned i = rels.size() - 2;
        for (; i != ~0u; i--) {
            auto r = rels[i];
            if (r == nextRoot)
                break;
            group[r] = 0;
            working0.push_back(r);
        }
        assert(rels[i] == nextRoot);
        assert(group[rels[i]] == 1);
        i--;
        for (; i != ~0u; i--) {
            auto r = rels[i];
            auto p = parents[r].parent;
            group[r] = group[p];
            (group[r] ? working1 : working0).push_back(r);
        }

        // Normalize
        ws[0].clear();
        ws[1].clear();
        for (unsigned r : ranges::reverse_view(working0))
            addToChain(links, ws[0], nodeCache[r]);
        for (unsigned r : ranges::reverse_view(working1))
            addToChain(links, ws[1], nodeCache[r]);
        // Merge chains
        auto merged = mergeChains(ws);
        merged.push_back(nodeCache[nextRoot]);
        auto modified = working0.size() + working1.size() + 1;
        modified = denormalize(links, merged, rels.subspan(rels.size() - modified));
        return modified;
    };
    auto buildLinks = [&]() {
        vector<Link> links;
        links.reserve(q.size());
        for (unsigned i = 0; i < q.size(); i++) links.push_back({i, i});
        return links;
    };
    auto buildAndDenormalize = [&](unsigned cur) -> SmallVector<unsigned> {
        auto links = buildLinks();
        nodeCache[cur] = Node{0.0, numeric_limits<float>::min(), cur};
        auto normalized = buildChain(links, cur);
        SmallVector<unsigned> denormalized;
        denormalized.resize(q.size());
        denormalize(links, normalized, denormalized);
        return denormalized;
    };
    auto computeCost = [&](span<const unsigned> rels) {
        float card = q.nodes[rels.back()].cardinality;
        float cost = 0.0;
        for (unsigned i = rels.size() - 2; i != ~0u; i--) {
            auto& p = parents[rels[i]];
            card *= q.nodes[rels[i]].cardinality * p.selectivity;
            cost += card;
        }
        return cost;
    };
    vector<vector<unsigned>> orders;
    orders.reserve(q.size());
    auto doRotations = Lambda::recursive([&](auto& self, span<Link> links, vector<unsigned>& rels, unsigned cur, unsigned parent = ~0u) -> void {
        for (auto child : adj[cur]) {
            if (child == parent) continue;
            auto& crels = orders.emplace_back(rels);
            rotate(links, crels, cur, child);

            if (opt.test) {
                auto other = buildAndDenormalize(child);
                if (crels != other) {
                    // Check if the costs are the same
                    if (abs(computeCost(crels) - computeCost(other)) > 1e-5) {
                        cerr << "Cost mismatch" << endl;
                        assertEq(crels, buildAndDenormalize(child), q);
                    }
                }
                if (q.size() < 12)
                    DP::optimizeLinear(mst, child, crels);
            }
            self(links, crels, child, cur);
            // Rotate back nodeCache
            parents[child].parent = cur;
            parents[child].selectivity = parents[cur].selectivity;
            nodeCache[cur] = Node{0.0, numeric_limits<float>::min(), cur};
            nodeCache[child] = computeNode(child);
        }
    });
    orders.push_back(buildAndDenormalize(startNode));
    auto links = buildLinks();
    doRotations(links, orders[0], startNode);

    if (opt.transfer == LinerizationTransfer::Sorted)
        sort(orders.begin(), orders.end());

    [[maybe_unused]] uint64_t totalReuse = 0;
    callback(orders[0]);
    for (size_t i = 1; i < orders.size(); i++) {
        unsigned j = 0;
        if (opt.transfer >= LinerizationTransfer::Basic)
            while (orders[i][j] == orders[i - 1][j])
                j++;
        totalReuse += j;
        callback(span{orders[i]}.subspan(j));
    }

    //cout << totalReuse * 1.0 / (q.size() * q.size()) << endl;
}
//---------------------------------------------------------------------------
}
//---------------------------------------------------------------------------