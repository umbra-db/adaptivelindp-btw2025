#include "Benchmark.hpp"
#include <iostream>
#include <cmath>
//---------------------------------------------------------------------------
using namespace std;
//---------------------------------------------------------------------------
namespace fastoptim {
//---------------------------------------------------------------------------
std::chrono::duration<double> Benchmark::measureInternal(FunctionRef<void(size_t repetitions)> f)
// Measure how much time a function takes to execute
{
    auto doMeasure = [&](size_t repetitions) {
        auto start = std::chrono::steady_clock::now();
        f(repetitions);
        auto end = std::chrono::steady_clock::now();
        return chrono::duration_cast<chrono::duration<double>>(end - start) / (1.0 * repetitions);
    };

    auto targetTime = 100ms;

    auto repetitions = 1;
    auto time = doMeasure(repetitions);
    auto lastTime = time;
    time = doMeasure(repetitions);

    // Try to reach target time and make sure the measurements are stable
    size_t steps = 0;
    while ((time * repetitions < targetTime) || (abs(time - lastTime) > 0.1 * time)) {
        repetitions = ceil(max(targetTime / time, max(repetitions * 1.1, repetitions + 1.0)));
        lastTime = time;
        time = doMeasure(repetitions);
        steps++;
    }

    return time;
}
//---------------------------------------------------------------------------
}
//---------------------------------------------------------------------------