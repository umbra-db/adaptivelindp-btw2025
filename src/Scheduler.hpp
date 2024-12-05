#pragma once
//---------------------------------------------------------------------------
#include <atomic>
#include <cassert>
#include <memory>
#include <string_view>
#include <vector>
#include "FunctionRef.hpp"
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
template <typename T = size_t>
struct Range {
    /// Start of range
    T st;
    /// End of range
    T en;
    /// Stride
    decltype(en - st) stride;
    /// Block size
    decltype(en - st) block;

    /// Default block count
    static constexpr decltype(en - st) defaultTargetBlockCount = 512;

    /// Pick a block
    static decltype(en - st) pickBlock(T st, T en, decltype(en - st) stride, size_t targetBlockCount = defaultTargetBlockCount) {
        auto blockSize = (en - st + targetBlockCount - 1) / targetBlockCount;
        // Block must be multiple of stride
        return (blockSize + stride - 1) / stride * stride;
    }

    /// Constructor
    constexpr Range(T st, T en, decltype(en - st) stride, decltype(en - st) block) noexcept : st(st), en(en), stride(stride), block(block) {
        assert((en - st) % stride == 0);
        assert(block % stride == 0);
    }
    /// Constructor
    constexpr Range(T st, T en, decltype(en - st) stride = 1) noexcept : st(st), en(en), stride(stride), block(pickBlock(st, en, stride)) {
        assert((en - st) % stride == 0);
        assert(block % stride == 0);
    }
    /// Constructor
    constexpr Range(T en) noexcept : Range(0, en) {}

    /// Number of blocks
    constexpr size_t count() const {
        return ((en - st) + block - 1) / block;
    }
};
//---------------------------------------------------------------------------
/// A scheduler
class Scheduler {
   struct Impl;
   /// The implementation
   std::unique_ptr<Impl> impl;

   public:
   /// Constructor
   explicit Scheduler(size_t numThreads = -1);
   /// Destructor
   ~Scheduler() noexcept;

   /// Run a morsel task on the scheduler
   void run(std::string_view taskName, FunctionRef<void()> task);
   /// Run a morsel task on the scheduler
   void run(FunctionRef<void()> task) { run("", task); }

   /// Get id of current thread
   static size_t getThreadId() noexcept;

   /// A zero state
   struct ZeroState {
      int operator()() const { return 0; }
   };

   /// Maximum number of threads
   static constexpr size_t maxThreadCount = 512;

   /// Task for parallel for
   template <typename T, typename TLState, typename Callback>
   struct ParallelForTask {
      struct alignas(64) ThreadState {
         T current;
         T end;
      };
      mutable std::vector<ThreadState> threadState;
      Range<T> fullRange;
      uint64_t threadCount;
      mutable Callback callback;
      mutable TLState tlState;

      /// Constructor
      ParallelForTask(Range<T> fullRange, size_t threadCount, Callback&& callback, TLState&& tlState) : fullRange(fullRange), threadCount(threadCount), callback(std::move(callback)), tlState(std::move(tlState)) {
         threadState.resize(threadCount);
         if (fullRange.en == fullRange.st) {
            threadState[0] = {fullRange.st, fullRange.st};
            return;
         }

         auto b = Range<T>::pickBlock(fullRange.st, fullRange.en, fullRange.block, threadCount);
         size_t ind = 0;
         for (auto st = fullRange.st; st < fullRange.en; st += b)
            threadState[ind++] = {st, st + b};
         assert(ind <= threadCount);
         assert(ind > 0);
         threadState[ind - 1].end = fullRange.en;
      }

      /// Pick morsels
      void operator()() const {
         auto localState = tlState();
         uint64_t tid = Scheduler::getThreadId();
         uint64_t originalTid = tid;
         do {
            std::atomic_ref<T> curRef(threadState[tid].current);
            auto cur = curRef.load();
            if (cur < threadState[tid].end) {
               cur = curRef.fetch_add(fullRange.block);
               while (cur < threadState[tid].end) {
                  auto curEnd = std::min<T>(threadState[tid].end, cur + fullRange.block);
                  for (auto i = cur; i < curEnd; i += fullRange.stride)
                     callback(localState, i);
                  cur = curRef.fetch_add(fullRange.block);
               }
            }

            tid++;
            if (tid >= threadCount)
               tid = 0;
         } while (tid != originalTid);
      }
   };

   /// Create a parallel for task
   template <typename T, typename Func, typename TLState = ZeroState>
   auto parallelForTask(Range<T> range, Func&& func, TLState&& tlState = {}) {
      ParallelForTask<T, TLState, Func> task(range, threadCount(), std::forward<Func>(func), std::forward<TLState>(tlState));

      return task;
   }

   /// Run a parallel for task
   template <typename T, typename Func, typename TLState = ZeroState>
   void parallelFor(std::string_view taskName, Range<T> range, Func&& func, TLState&& tlState = {}) {
      auto task = parallelForTask(range, std::forward<Func>(func), std::forward<TLState>(tlState));
      run(taskName, task);
   }

   /// Run a parallel for task
   template <typename T, typename Func, typename TLState = ZeroState>
   void parallelFor(Range<T> range, Func&& func, TLState&& tlState = {}) {
      auto task = parallelForTask(range, std::forward<Func>(func), std::forward<TLState>(tlState));
      run(task);
   }

   /// Get the number of threads
   size_t threadCount();

    /// Get the number of hardware threads
   static size_t hardwareThreads();
};
//---------------------------------------------------------------------------
}
