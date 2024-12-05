#include "Benchmark.hpp"
#include "BumpAlloc.hpp"
#include "CSV.hpp"
#include "DP.hpp"
#include "IKKBZ.hpp"
#include "LinDP.hpp"
#include "QueryGraph.hpp"
#include "PrintUtil.hpp"
#include "Plan.hpp"
#include "Random.hpp"
#include "Scheduler.hpp"
#include "RMQ.hpp"
#include "MDQ.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <filesystem>
#include <format>
#include <fstream>
#include <numeric>
#include <bitset>
#include <functional>
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
/*template <typename T>
std::strong_ordering operator<=>(std::span<T> lhs, std::span<T> rhs) {
    for (size_t i = 0; i < std::min(lhs.size(), rhs.size()); ++i) {
        if (auto cmp = lhs[i] <=> rhs[i]; cmp != 0) {
            return cmp;
        }
    }
    return lhs.size() <=> rhs.size();
}*/
//---------------------------------------------------------------------------
std::vector<std::tuple<unsigned, unsigned, unsigned, float>> mcm(const QueryGraph& g, bool prep = true) {
    std::vector<std::tuple<unsigned, unsigned, unsigned, float>> result;
    IncrementalDP dp(g);
    vector<unsigned> nodes(g.size());
    for (unsigned i = 0; i < g.size(); i++)
        nodes[i] = g.size() - 1 - i;
    vector<char> vis;
    if (prep)
        vis.resize(g.size() * g.size());
    dp.updateOld(nodes, [&](unsigned i, unsigned k, unsigned j, float sel) {
        if (prep) {
            if (vis[i * g.size() + j - i]) sel = -1.0f;
            vis[i * g.size() + j - i] = 1;
            result.emplace_back(i, k, j, sel);
        }
    });
    if (prep)
        sort(result.begin(), result.end());
    return result;
}
//---------------------------------------------------------------------------
std::vector<std::tuple<unsigned, unsigned, unsigned, float>> eff(const QueryGraph& g, bool prep = true) {
    std::vector<std::tuple<unsigned, unsigned, unsigned, float>> result;
    IncrementalDP dp(g);
    vector<unsigned> nodes(g.size());
    for (unsigned i = 0; i < g.size(); i++)
        nodes[i] = g.size() - 1 - i;
    vector<char> vis;
    if (prep)
        vis.resize(g.size() * g.size());
    dp.updateNew(nodes, [&](unsigned i, unsigned k, unsigned j, float sel) {
        if (prep) {
            if (vis[i * g.size() + j - i]) sel = -1.0f;
            vis[i * g.size() + j - i] = 1;
            result.emplace_back(i, k, j, sel);
        }
    });
    if (prep)
        sort(result.begin(), result.end());
    return result;
}
//---------------------------------------------------------------------------
std::vector<std::vector<unsigned>> ikkbz(const QueryGraph& g, bool test = false) {
    std::vector<std::vector<unsigned>> result;
    IKKBZ::optimize(g, [&](span<const unsigned> diff) {
        if (result.empty()) {
            result.push_back(vector<unsigned>(diff.begin(), diff.end()));
        } else {
            result.push_back(result.back());
            copy(diff.begin(), diff.end(), result.back().begin() + (result.back().size() - diff.size()));
        }
    }, IKKBZOptions{.test = test});
    sort(result.begin(), result.end());
    return result;
}
//---------------------------------------------------------------------------
#define check(B)  do { if (!(B)) { std::cerr << "Check failed: " << #B << std::endl; exit(1); } } while(false)
//---------------------------------------------------------------------------
void testRQ() {
    RMQ rmq(10);
    for (int i = 0; i < 10; i++) {
        rmq.update(i, i);
    }
    for (int i = 0; i < 10; i++) {
        check(rmq.query(i, i) == i);
        check(rmq.query(i, 9) == i);
        check(rmq.query(0, i) == 0);
    }

    MDQ<int32_t> rq(10);
    for (int i = 0; i < 10; i++) {
        rq.update(i, i);
    }
    for (int i = 0; i < 10; i++) {
        auto res = rq.query(i, i);
        check(res == i);
    }
    rq.update(5, 3);
    rq.update(7, 1);
    check(rq.query(4, 3) == 5);
    check(rq.query(4, 2) == 7);
    check(rq.query(5, 3) == 5);
    check(rq.query(6, 3) == 7);

}
//---------------------------------------------------------------------------
void testIKKBZ() {
    QueryGraph q = QueryGraph::fromLists({{0, 1, 0.1}, {0, 2, 0.2}, {0, 3, 0.3}});
    IKKBZ::optimize(q, [](span<const unsigned> s) {
        std::cout << "diff" << s << std::endl;
    });
}
//---------------------------------------------------------------------------
/// Printer for chrono
struct CP {
    chrono::duration<double> dur;
};
//---------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& out, CP cp) {
    // Switch unit based on duration
    auto oldPrec = out.precision();
    out.precision(2);
    if (cp.dur < 1us) {
        out << chrono::duration_cast<chrono::nanoseconds>(cp.dur).count() << "ns";
    } else if (cp.dur < 1ms) {
        out << chrono::duration_cast<chrono::microseconds>(cp.dur).count() << "us";
    } else if (cp.dur < 1s) {
        out << chrono::duration_cast<chrono::milliseconds>(cp.dur).count() << "ms";
    } else if (cp.dur < 1min) {
        out << chrono::duration_cast<chrono::seconds>(cp.dur).count() << "s";
    } else {
        out << chrono::duration_cast<chrono::minutes>(cp.dur).count() << "m";
    }
    out.precision(oldPrec);
    return out;
}
//---------------------------------------------------------------------------
template <bool prepResult = true>
vector<pair<uint64_t, uint64_t>> testLinDP(const QueryGraph& qg, LinDPOptions opt, bool dedup = true) {
    struct Plan : ::fastoptim::Plan {
        /// Constructor
        float cardinality = 0.0f;
        uint64_t bs = 0;
    };
    BumpAlloc<Plan> alloc;
    vector<fastoptim::Plan*> basePlans;
    basePlans.reserve(qg.size());
    for (unsigned i = 0; i < qg.size(); i++) {
        auto* p = &alloc.emplace();
        basePlans.push_back(p);
        p->cardinality = qg.nodes[i].cardinality;
        p->cost = 0.0f;
        p->bs = 1ull << (i % 64);
    }

    vector<pair<uint64_t, uint64_t>> result;

    auto hashBS = [](uint64_t bs) {
        return bs;
    };

    auto combine = [&](fastoptim::Plan* targetr, fastoptim::Plan* leftr, fastoptim::Plan* rightr, float sel) -> fastoptim::Plan* {
        auto* target = static_cast<Plan*>(targetr);
        auto* left = static_cast<Plan*>(leftr);
        auto* right = static_cast<Plan*>(rightr);
        if (!target) {
            target = alloc.allocate();
            target->cardinality = left->cardinality * sel * right->cardinality;
        }
        target->cost = left->cost + right->cost + target->cardinality;
        target->bs = left->bs | right->bs;
        if constexpr (prepResult)
            result.emplace_back(hashBS(left->bs), hashBS(right->bs));
        return target;
    };

    [[maybe_unused]] auto best = LinDP::optimize(qg, basePlans, combine, opt);

    if constexpr (prepResult) {
        sort(result.begin(), result.end());
        if (dedup)
            result.erase(unique(result.begin(), result.end()), result.end());
    }

    return result;
}
//---------------------------------------------------------------------------
vector<const char*> trees{
    //"data/tree3.txt",
    "data/tree4.2.txt","data/tree4.3.txt","data/tree5.2.txt","data/tree5.3.txt","data/tree5.4.txt","data/tree6.2.txt","data/tree6.3.txt","data/tree6.4.txt","data/tree6.5.txt","data/tree7.2.txt","data/tree7.3.txt","data/tree7.4.txt","data/tree7.5.txt","data/tree7.6.txt","data/tree8.2.txt","data/tree8.3.txt","data/tree8.4.txt","data/tree8.5.txt","data/tree8.6.txt","data/tree8.7.txt","data/tree9.2.txt","data/tree9.3.txt","data/tree9.4.txt","data/tree9.5.txt","data/tree9.6.txt","data/tree9.7.txt","data/tree9.8.txt","data/tree10.2.txt","data/tree10.3.txt","data/tree10.4.txt","data/tree10.5.txt","data/tree10.6.txt","data/tree10.7.txt","data/tree10.8.txt","data/tree10.9.txt","data/tree11.2.txt","data/tree11.3.txt","data/tree11.4.txt","data/tree11.5.txt","data/tree11.6.txt","data/tree11.7.txt","data/tree11.8.txt","data/tree11.9.txt","data/tree11.10.txt","data/tree12.2.txt","data/tree12.3.txt","data/tree12.4.txt","data/tree12.5.txt","data/tree12.6.txt","data/tree12.7.txt","data/tree12.8.txt","data/tree12.9.txt","data/tree12.10.txt","data/tree12.11.txt","data/tree13.2.txt","data/tree13.3.txt","data/tree13.4.txt","data/tree13.5.txt","data/tree13.6.txt","data/tree13.7.txt","data/tree13.8.txt","data/tree13.9.txt","data/tree13.10.txt","data/tree13.11.txt","data/tree13.12.txt","data/tree14.2.txt","data/tree14.3.txt","data/tree14.4.txt","data/tree14.5.txt","data/tree14.6.txt","data/tree14.7.txt","data/tree14.8.txt","data/tree14.9.txt","data/tree14.10.txt","data/tree14.11.txt","data/tree14.12.txt","data/tree14.13.txt","data/tree15.2.txt","data/tree15.3.txt","data/tree15.4.txt","data/tree15.5.txt","data/tree15.6.txt","data/tree15.7.txt","data/tree15.8.txt","data/tree15.9.txt","data/tree15.10.txt","data/tree15.11.txt","data/tree15.12.txt","data/tree15.13.txt","data/tree15.14.txt","data/tree16.2.txt","data/tree16.3.txt","data/tree16.4.txt","data/tree16.5.txt","data/tree16.6.txt","data/tree16.7.txt","data/tree16.8.txt","data/tree16.9.txt","data/tree16.10.txt","data/tree16.11.txt","data/tree16.12.txt","data/tree16.13.txt","data/tree16.14.txt","data/tree16.15.txt","data/tree17.2.txt","data/tree17.3.txt","data/tree17.4.txt","data/tree17.5.txt","data/tree17.6.txt","data/tree17.7.txt","data/tree17.8.txt","data/tree17.9.txt","data/tree17.10.txt","data/tree17.11.txt","data/tree17.12.txt","data/tree17.13.txt","data/tree17.14.txt","data/tree17.15.txt","data/tree17.16.txt","data/tree18.2.txt","data/tree18.3.txt","data/tree18.4.txt","data/tree18.5.txt","data/tree18.6.txt","data/tree18.7.txt","data/tree18.8.txt","data/tree18.9.txt","data/tree18.10.txt","data/tree18.11.txt","data/tree18.12.txt","data/tree18.13.txt","data/tree18.14.txt","data/tree18.15.txt","data/tree18.16.txt","data/tree18.17.txt","data/tree19.2.txt","data/tree19.3.txt","data/tree19.4.txt","data/tree19.5.txt","data/tree19.6.txt","data/tree19.7.txt","data/tree19.8.txt","data/tree19.9.txt","data/tree19.10.txt","data/tree19.11.txt","data/tree19.12.txt","data/tree19.13.txt","data/tree19.14.txt","data/tree19.15.txt","data/tree19.16.txt","data/tree19.17.txt","data/tree19.18.txt","data/tree20.2.txt","data/tree20.3.txt","data/tree20.4.txt","data/tree20.5.txt","data/tree20.6.txt","data/tree20.7.txt","data/tree20.8.txt","data/tree20.9.txt","data/tree20.10.txt","data/tree20.11.txt","data/tree20.12.txt","data/tree20.13.txt","data/tree20.14.txt","data/tree20.15.txt","data/tree20.16.txt","data/tree20.17.txt","data/tree20.18.txt","data/tree20.19.txt","data/tree21.2.txt","data/tree21.3.txt","data/tree21.4.txt","data/tree21.5.txt","data/tree21.6.txt","data/tree21.7.txt","data/tree21.8.txt","data/tree21.9.txt","data/tree21.10.txt","data/tree21.11.txt","data/tree21.12.txt","data/tree21.13.txt","data/tree21.14.txt","data/tree21.15.txt","data/tree21.16.txt","data/tree21.17.txt","data/tree21.18.txt","data/tree21.19.txt","data/tree21.21.txt",
};
//---------------------------------------------------------------------------
struct TestData {
    string_view category;
    std::function<QueryGraph(unsigned)> gen;
    double param0 = 0.0;
    double param1 = 0.0;

    template <typename F>
    TestData(string_view category, F&& gen, double param0 = 0.0, double param1 = 0.0) : category(category), gen(std::forward<F>(gen)), param0(param0), param1(param1) {}
};
//---------------------------------------------------------------------------
vector<TestData> genTestData()
// Generate test graphs
{
    vector<TestData> result;

    result.push_back({"clique"sv, [](unsigned n) {
        vector<vector<unsigned>> adj(n);
        for (unsigned i = 0; i < n - 1; i++)
            for (unsigned j = i + 1; j < n; j++)
                adj[i].push_back(j);
        return QueryGraph::fromAdjList(adj);
    }});
    result.push_back({"chain"sv, [](unsigned n) {
        vector<vector<unsigned>> adj(n);
        for (unsigned i = 0; i < n - 1; i++)
            adj[i].push_back(i + 1);
        return QueryGraph::fromAdjList(adj);
    }});
    result.push_back({"star"sv, [](unsigned n) {
        vector<vector<unsigned>> adj(n);
        for (unsigned i = 1; i < n; i++)
            adj[0].push_back(i);
        return QueryGraph::fromAdjList(adj);
    }});
    double density = 0.0;
    for (size_t cl = 0; cl <= 10; cl++) {
        double chainLen = cl / 10.0;
        for (size_t seed = 0; seed < 5; seed++) {
            result.push_back({"tree"sv, [seed, chainLen, density](unsigned n) {
                Random rng(1024 + seed);
                return QueryGraph::randomTree(n, chainLen, density, rng);
            }, chainLen, density});
        }
    }
    return result;
}
//---------------------------------------------------------------------------
struct Algo {
    string_view type;
    LinDPOptions opt;
    std::function<void(const QueryGraph&)> func;
};
//---------------------------------------------------------------------------
vector<Algo> collectAlgorithms() {
    vector<Algo> result;
    auto doLinDP = [&](LinDPOptions opt) {
        result.push_back({"lindp"sv, opt, [opt](const QueryGraph& q) { testLinDP<false>(q, opt, false); }});
    };
    auto doOther = [&](string_view type, FunctionRef<void(const QueryGraph&)> func) {
        result.push_back({type, {}, func});
    };

    doLinDP(LinDPOptions{.ikkbz = { .transfer = LinerizationTransfer::None }, .dp = { .newAlgo = false }});
    doLinDP(LinDPOptions{.ikkbz = { .transfer = LinerizationTransfer::Basic }, .dp = { .newAlgo = false }});
    doLinDP(LinDPOptions{.ikkbz = { .transfer = LinerizationTransfer::Sorted }, .dp = { .newAlgo = false }});
    doLinDP(LinDPOptions{.ikkbz = { .transfer = LinerizationTransfer::None }, .dp = { .newAlgo = true }});
    doLinDP(LinDPOptions{.ikkbz = { .transfer = LinerizationTransfer::Basic }, .dp = { .newAlgo = true }});
    doLinDP(LinDPOptions{.ikkbz = { .transfer = LinerizationTransfer::Sorted }, .dp = { .newAlgo = true }});

    doOther("mcm"sv, [](const QueryGraph& q) { mcm(q, false); });
    doOther("eff"sv, [](const QueryGraph& q) { eff(q, false); });

    return result;
}
//---------------------------------------------------------------------------
/// Helper for progress resports
struct Progress {
    /// Start time
    chrono::steady_clock::time_point st;
    /// Current finished task count
    atomic<size_t> cur{};
    /// Total task count
    size_t total = 0;
    /// Last printed string
    string lastprint;
    /// Lock
    mutex lock;

    /// Constructor
    Progress(size_t newTotal = 0) {
        reset(newTotal);
    }

    /// Reset progress
    void reset(size_t newTotal) {
        st = chrono::steady_clock::now();
        total = newTotal;
        cur.store(0);
    }

    /// Print progress
    void print(string_view content = "") {
        size_t d = cur.load();
        size_t dist = total;
        auto cur = chrono::steady_clock::now();
        auto elapsed = cur - st;
        auto ratio = d * 1.0 / static_cast<double>(dist);
        auto tosecs = [](auto d) { return chrono::seconds(chrono::duration_cast<chrono::seconds>(d).count()); };
        auto formatTime = [&](chrono::seconds dur) {
            auto secs = dur.count();
            if (dur.count() < 24 * 60 * 60)
                return format("{:%H:%M:%S}", dur);
            else
                return format("{}d {:%H:%M:%S}", secs / (24 * 60 * 60), dur);
        };
        auto newprint = format("[Elapsed:{} | Remaining:{}] Progress {:.1f}% : {}/{} | {}", formatTime(tosecs(elapsed)), formatTime(tosecs(elapsed / ratio - elapsed)), ratio * 100, d, dist, content);

        unique_lock ul(lock);
        cout << "\r" << newprint << string((lastprint.size() > newprint.size()) ? lastprint.size() - newprint.size() : 0, ' ');
        cout.flush();
        swap(lastprint, newprint);
    }

    /// Increment done counter
    void inc(size_t done) {
        cur.fetch_add(done);
    }
};
//---------------------------------------------------------------------------
static void progressFor(Scheduler& s, string_view taskName, Range<> r, auto&& lamb) {
    [[maybe_unused]] auto normalFor = [](const auto& range, auto&& func) {
        auto zero = Scheduler::ZeroState{};
        for (auto i = range.st; i < range.en; ++i)
            func(zero, i);
    };
    Progress progress(r.en - r.st);
    progress.print(taskName);
    s.parallelFor(r, [&](auto&& state, size_t i) {
        lamb(state, i);
        progress.inc(1);
        progress.print(taskName);
    });
    cout << endl;
}
//---------------------------------------------------------------------------
void doBenchmark(bool justCheck = false) {
    auto data = genTestData();
    auto algos = collectAlgorithms();


    auto doLinDP = [&](const QueryGraph& q, LinDPOptions opt, bool dedup = false) {
        return testLinDP(q, opt, dedup);
    };

    Scheduler s(max<size_t>(2, 1));

    // Correctness checks
    /*progressFor(s, "check", Range{data.size()}, [&](auto&&, size_t i) {
        auto& d = data[i];
        size_t lim = 50;
        for (unsigned n = 3; n < lim; n++) {
            Random rng(i * lim + n);
            auto q = d.gen(n);
            size_t totTrials = 10;
            for (size_t trial = 0; trial < totTrials; trial++) {
                q.randomizeWeights(rng);
                assert(q.nodes.size() == n);

                {
                    // Correctness checks
                    auto on = doLinDP(q, {.ikkbz = {.test = true, .transfer = LinerizationTransfer::None}, .dp = {.newAlgo = false}}, false);
                    {
                        auto nn = doLinDP(q, {.ikkbz = {.test = true, .transfer = LinerizationTransfer::None}, .dp = {.newAlgo = true}}, false);
                        assertEq(on, nn, q);
                    }
                    on.erase(unique(on.begin(), on.end()), on.end());
                    {
                        auto ob = doLinDP(q, {.ikkbz = {.test = true, .transfer = LinerizationTransfer::Basic}, .dp = {.newAlgo = false}}, false);
                        {
                            auto nb = doLinDP(q, {.ikkbz = {.test = true, .transfer = LinerizationTransfer::Basic}, .dp = {.newAlgo = true}}, false);
                            assertEq(ob, nb, q);
                        }
                        ob.erase(unique(ob.begin(), ob.end()), ob.end());
                        assertEq(on, ob, q);
                    }
                    {
                        auto os = doLinDP(q, {.ikkbz = {.test = true, .transfer = LinerizationTransfer::Sorted}, .dp = {.newAlgo = false}}, false);
                        {
                            auto ns = doLinDP(q, {.ikkbz = {.test = true, .transfer = LinerizationTransfer::Sorted}, .dp = {.newAlgo = true}}, false);
                            assertEq(os, ns, q);
                        }
                        os.erase(unique(os.begin(), os.end()), os.end());
                        assertEq(on, os, q);
                    }
                }

                {
                    auto m = mcm(q, true);
                    auto e = eff(q, true);
                    // No guarantees are given regarding edge selection if graph is cyclic
                    if (d.param1 != 0) {
                        for (auto& [a, b, c, d] : m)
                            d = 0.0;
                        for (auto& [a, b, c, d] : e)
                            d = 0.0;
                    }
                    assertEq(m, e, q);
                }
            }
        }
    });
    cout << "Done check" << endl;*/

    if (justCheck)
        return;

    filesystem::create_directory("output");
    ofstream out("output/benchmark.csv");
    CSV<string_view, size_t, size_t, double, double, string_view, string_view, string_view, double> csv(out, {"graphType"s, "n"s, "m"s, "chainLen"s, "density"s, "algoType"s, "transfer"s, "dpkind"s, "time"s}, ",");
    csv.header();

    progressFor(s, "bench", Range<size_t>{0, data.size() * algos.size(), 1, 1}, [&](auto&&, size_t idx) {
        auto di = idx / algos.size();
        auto ai = idx % algos.size();
        auto& d = data[di];
        const auto& a = algos[ai];
        auto nextN = [](unsigned n) -> unsigned {
            size_t lim = 16;
            if (n < lim)
                return n + 1;
            return ceil(n * (1.0 + 1.0 / lim));
        };
        for (unsigned n = 3; ; n = nextN(n)) {
            Random rng(n * data.size() + di);
            auto q = d.gen(n);
            chrono::duration<double> avgdur{};
            size_t totTrials = 5;
            for (size_t trial = 0; trial < totTrials; trial++) {
                q.randomizeWeights(rng);
                auto dur = Benchmark::measure([&] { a.func(q); });
                csv.ln(d.category, q.size(), q.edges.size(), d.param0, d.param1, a.type, a.opt.ikkbz.transfer == LinerizationTransfer::None ? "none"s : a.opt.ikkbz.transfer == LinerizationTransfer::Basic ? "basic"s : "sorted"s, a.opt.dp.newAlgo ? "new" : "old", dur.count());
                avgdur += dur / totTrials;
            }
            // Grow until we reach 1s
            if (avgdur >= 1s)
                break;
        }
    });
    cout << "Done bench" << endl;
}
//---------------------------------------------------------------------------
void testLinDPSmall() {
    QueryGraph q = QueryGraph::fromLists({{0, 1, 0.1}, {1, 2, 0.2}, {0, 2, 0.3}, {2, 3, 0.5}});
    auto n0 = 1;
    auto n1 = 2;
    auto n2 = 4;
    auto n3 = 8;
    auto n01 = 3;
    auto n12 = 6;
    auto n02 = 5;
    auto n23 = 12;
    auto n123 = 14;
    auto n023 = 13;
    auto n012 = 7;
    auto n0123 = 15;
    // Connected component pairs
    vector<pair<uint64_t, uint64_t>> all {
        // 0 1 2 3
        {n2, n3}, {n1, n2}, {n1, n23}, {n12, n3}, {n0, n1}, {n0, n12}, {n0, n123}, {n01, n2}, {n01, n23}, {n012, n3},
        // 1 0 2 3
        {n2, n3}, {n0, n2}, {n0, n23}, {n02, n3}, {n1, n0}, {n1, n02}, {n1, n023}, {n01, n2}, {n01, n23}, {n012, n3},
        // 2 1 0 3
        // 3 2 1 0
    };

    auto printVal = [](uint64_t v) {
        for (unsigned i = 0; i < 3 - popcount(v); i++) cout << " ";
        for (unsigned i = 0; i < 4; i++) {
            if (v & (1ull << i)) cout << i;
        }
    };
    auto printVec = [&](vector<pair<uint64_t, uint64_t>>& v) {
        cout << "[";
        for (auto& p : v) {
            cout << "(";
            printVal(p.first);
            cout << ", ";
            printVal(p.second);
            cout << ") ";
        }
        cout << "]";
        cout << endl;
    };

    auto oldvn = testLinDP(q, { .ikkbz = {.transfer = LinerizationTransfer::None}, .dp = {. newAlgo = false} }, false);
    auto oldvb = testLinDP(q, { .ikkbz = {.transfer = LinerizationTransfer::Basic}, .dp = {. newAlgo = false} }, false);
    auto oldvs = testLinDP(q, { .ikkbz = {.transfer = LinerizationTransfer::Sorted}, .dp = {. newAlgo = false} }, false);
    auto newvn = testLinDP(q, { .ikkbz = {.transfer = LinerizationTransfer::None}, .dp = {. newAlgo = false} }, false);
    auto newvb = testLinDP(q, { .ikkbz = {.transfer = LinerizationTransfer::Basic}, .dp = {. newAlgo = false} }, false);
    auto newvs = testLinDP(q, { .ikkbz = {.transfer = LinerizationTransfer::Sorted}, .dp = {. newAlgo = false} }, false);
    //printVec(oldv);
    //printVec(newv);
    assertEq(newvn.size(), oldvn.size());
    assertEq(oldvb.size(), oldvn.size() - 1);
    assertEq(oldvs.size(), oldvn.size() - 1);
    assertEq(newvb.size(), oldvn.size() - 1);
    assertEq(newvs.size(), oldvn.size() - 1);

    sort(oldvn.begin(), oldvn.end());
    sort(oldvb.begin(), oldvb.end());
    sort(oldvs.begin(), oldvs.end());
    sort(newvn.begin(), newvn.end());
    sort(newvb.begin(), newvb.end());
    sort(newvs.begin(), newvs.end());

    assertEq(newvn, oldvn);
    assertEq(newvb, oldvb);
    assertEq(newvs, oldvs);
}
//---------------------------------------------------------------------------
int mainImpl(int argc, char* argv[]) {
    std::cout << "Hello, World!" << std::endl;
    testLinDPSmall();
    testRQ();
    testIKKBZ();
    //testGraphTypes("old", [](const QueryGraph& qg, bool prep) { return prep ? testLinDP<true>(qg, true) : testLinDP<false>(qg, true); }, "new", [](const QueryGraph& qg, bool prep) { return prep ? testLinDP<true>(qg, false) : testLinDP<false>(qg, false); });
    //testGraphTypes("mcm", mcm, "eff", eff);

    // graph G {0--3[label="0.878841"];1--3[label="0.672048"];2--3Error: [0, 3, [label="0.236101"];}
    QueryGraph q = QueryGraph::fromLists({{0, 3, 0.878841}, {1, 3, 0.672048}, {2, 3, 0.236101}});
    IKKBZ::optimize(q, [](span<const unsigned> s) {
        std::cout << "diff" << s << std::endl;
    }, { .test = true });

    /*Random rng;
    rng = Random(1024);
    auto t0 = QueryGraph::randomTree(278, 1.0, 0.0, rng);
    rng = Random(1024);
    auto t1 = QueryGraph::randomTree(296, 1.0, 0.0, rng);

    cout << t0 << endl;
    cout << t1 << endl;

    for (size_t i = 0; i < 10; i++) {
        t0.randomizeWeights(rng);
        t1.randomizeWeights(rng);
        auto computeShared = [](const QueryGraph& qg) {
            double shared = 0;
            IKKBZ::optimize(qg, [&](span<const unsigned> changed) {
                shared += (qg.size() - changed.size()) * 1.0 / qg.size();
            });
            return shared / qg.size();
        };

        cout << "t0: " << computeShared(t0) << " | t1: " << computeShared(t1) << endl;
    }
    exit(0);*/

    bool print = false;

    auto doGraph = [&](std::string_view s) {
        if (print) {
            std::cout << "----" << std::endl;
            std::cout << s << std::endl;
        }
        QueryGraph g = QueryGraph::fromGraph6(s);
        if (print) std::cout << g << std::endl;
        auto m = mcm(g);
        if (print) std::cout << m << std::endl;
        auto e = eff(g);
        if (print) std::cout << e << std::endl;
        if (m != e) {
            std::cout << "Mismatch for " << s << std::endl;
            std::cout << g << std::endl;
            std::cout << "MCM: " << m << std::endl;
            std::cout << "EFF: " << e << std::endl;
            exit(1);
        }
    };

    size_t seed = 1;
    auto doTree = [&](std::string_view s) {
        if (print) {
            std::cout << "----" << std::endl;
            std::cout << s << std::endl;
        }
        QueryGraph g = QueryGraph::fromEdges(s);
        auto ss = seed++;
        Random rng(ss);
        g.randomizeWeights(rng);
        if (print) std::cout << g << std::endl;
        try {
            auto m = ikkbz(g, true);
            if (print) std::cout << m << std::endl;
        } catch (...) {
            std::cerr << "Seed: " << ss << std::endl;
            std::cerr << g << std::endl;
            exit(1);
        }
    };


    if (argc > 1) {
        doGraph(argv[1]);
        return 0;
    }

    //Scheduler s;
    //progressFor(s, "trees", Range{trees.size()}, [&](auto&&, size_t i) {
    //    const auto& fname = trees[i];
    //    if (!std::filesystem::exists(fname)) {
    //        std::cerr << "File " << fname << " does not exist" << std::endl;
    //        return;
    //    }
    //    std::ifstream f(fname);
    //    std::string s;
    //    while (std::getline(f, s)) {
    //        doTree(s);
    //    }
    //});
    //cout << "Tested trees" << endl;
    //progressFor(s, "graphs", Range<size_t>{2, 11}, [&](auto&&, size_t i) {
    //    auto fname = std::format("data/graph{}c.g6", i);
    //    if (!std::filesystem::exists(fname)) {
    //        std::cerr << "File " << fname << " does not exist" << std::endl;
    //        return;
    //    }
    //    std::ifstream f(fname);
    //    std::string s;
    //    while (std::getline(f, s)) {
    //        doGraph(s);
    //    }
    //});
    //cout << "Tested graphs" << endl;

    doBenchmark(false);

    return 0;
}
//---------------------------------------------------------------------------
}
//---------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    return fastoptim::mainImpl(argc, argv);
}
