#pragma once
//---------------------------------------------------------------------------
#include <span>
#include <string_view>
#include <tuple>
#include <vector>
#include <iosfwd>
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
class Random;
//---------------------------------------------------------------------------
/// A query graph encoding of a join query
class QueryGraph {
    public:
    /// A node/relation
    struct Node {
        /// The relation's cardinality
        float cardinality = 1.0;
    };
    /// An edge
    struct Edge {
        /// The target node
        unsigned target = 0;
        /// The edge's selectivity
        float selectivity = 1.0;
    };
    /// The nodes
    std::vector<Node> nodes;
    /// The edges
    std::vector<Edge> edges;
    /// The ranges of edges for every node
    std::vector<unsigned> ranges;

    /// Get the outgoing edges to nodes with larger indices
    [[nodiscard]] constexpr std::span<const Edge> getEdges(unsigned node) const {
        auto begin = ranges[node];
        auto end = ranges[node + 1];
        return {edges.data() + begin, end - begin};
    }
    /// Get the outgoing edges to nodes with larger indices
    [[nodiscard]] constexpr std::span<Edge> getEdges(unsigned node) {
        auto begin = ranges[node];
        auto end = ranges[node + 1];
        return {edges.data() + begin, end - begin};
    }

    /// Read graph encoded in Graph6 format. Set cardinalities and selectivities to 1
    static QueryGraph fromGraph6(std::string_view s);
    /// Encode graph
    std::string toGraph6() const;
    /// Read graph encoded in edges format. Set cardinalities and selectivities to 1
    static QueryGraph fromEdges(std::string_view s);
    /// Build graph from adjacency list
    static QueryGraph fromAdjList(std::span<const std::vector<unsigned>> adjList, std::span<const float> relations = {});
    /// Build graph from adjacency list
    static QueryGraph fromAdjList(std::span<const std::vector<std::pair<unsigned, float>>> adjList, std::span<const float> relations = {});
    /// Build graph from edge and relation list
    static QueryGraph fromLists(std::span<const std::tuple<unsigned, unsigned, float>> edges, std::span<const float> relations);
    /// Build graph from edge and relation list
    template <size_t N1, size_t N2>
    static QueryGraph fromLists(const std::tuple<unsigned, unsigned, float> (&edges)[N1], const float (&relations)[N2]) {
        return fromLists(std::span{edges}, std::span{relations});
    }
    /// Build graph from edge and relation list
    template <size_t N1>
    static QueryGraph fromLists(const std::tuple<unsigned, unsigned, float> (&edges)[N1]) {
        return fromLists(std::span{edges}, {});
    }
    /// Generate a random tree on top of a chain. Set cardinalities and selectivities to 1
    static QueryGraph randomTree(unsigned n, double chainRatio, double density, Random& eng);

    /// Compute the minimum spanning tree of the graph
    QueryGraph computeMST() const;
    /// Compute the reverse of the graph
    QueryGraph computeReverse() const;

    /// Randomize weights
    void randomizeWeights(Random& eng);

    /// The number of nodes
    constexpr size_t size() const noexcept { return nodes.size(); }
};
//---------------------------------------------------------------------------
/// Write graph out in dot format
std::ostream& operator<<(std::ostream& out, const QueryGraph& q);
//---------------------------------------------------------------------------
}