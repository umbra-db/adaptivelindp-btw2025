#include "Scheduler.hpp"
#include "Benchmark.hpp"
#include <atomic>
#include <cassert>
#include <thread>
#include <mutex>
#include <queue>
#include <chrono>
#include <latch>
#include <condition_variable>
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
#include <immintrin.h>
#endif
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
thread_local size_t localThreadId = 0;
//---------------------------------------------------------------------------
struct Task {
   FunctionRef<void()> func{};
   size_t taskId = 0;
   atomic<int64_t> workerCount{};
   mutex mut;
   condition_variable workDone;
};
//---------------------------------------------------------------------------
static constexpr size_t spinCount = 40;
//---------------------------------------------------------------------------
void hardwarePause() {
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
    _mm_pause();
#elif defined(__aarch64__) || defined(__arm__)
    asm volatile("yield" ::: "memory");
#else
    asm volatile("" ::: "memory");
#endif
}
//---------------------------------------------------------------------------
struct Scheduler::Impl {
   size_t totalThreads;
   unique_ptr<Task> task;
   size_t lastTaskId = 0;
   latch finalKilled;


   condition_variable workdoneCv;

   struct Worker {
      size_t threadId;
      Impl* impl;

      mutex mut;
      atomic<Task*> task{nullptr};
      condition_variable workAvailable;
      jthread thr;

      void work(stop_token tok) {
         while (!tok.stop_requested()) {
            Task* currentTask = nullptr;
            for (size_t i = 0; !task.load() && (i < spinCount); i++)
                hardwarePause();
            currentTask = task.exchange(currentTask);
            if (!currentTask) {
               unique_lock ul(mut);
               if (!task.load()) {
                  //fmt::print("Waiting for work {}\n", threadId);
                  workAvailable.wait(ul, [&]() -> bool { return (task.load()) || tok.stop_requested(); });
               }
               currentTask = task.exchange(currentTask);
               if (tok.stop_requested())
                  break;
            }
            currentTask->func();
            auto v = currentTask->workerCount.fetch_sub(1);
            assert(v > 0);
            if (v == 1) {
               //fmt::print("Report done {}\n", threadId);
               unique_lock ul(currentTask->mut);
               currentTask->workDone.notify_all();
               currentTask->workerCount.fetch_sub(1);
            }
         }
         impl->finalKilled.count_down();
      }

      explicit Worker(Impl& impl, size_t tid) : threadId(tid), impl(&impl), thr([this, tid](stop_token tok) {
                                                   localThreadId = tid;
                                                   this->work(move(tok));
                                                }) {}

      ~Worker() noexcept {
         //fmt::print("Destroying {}\n", threadId);
         thr.join();
      }
   };

   vector<unique_ptr<Worker>> threads;


   explicit Impl(size_t numThreads) : totalThreads(numThreads == -1 ? thread::hardware_concurrency() : numThreads), finalKilled(totalThreads) {
      threads.reserve(totalThreads);
      for (size_t i = 0; i < totalThreads; i++) {
         threads.push_back(make_unique<Worker>(*this, i));
      }
   }

   ~Impl() noexcept {
      for (auto& w : threads) {
         w->thr.request_stop();
         unique_lock lg(w->mut);
         w->workAvailable.notify_one();
      }
      finalKilled.wait();
   }
};
//---------------------------------------------------------------------------
Scheduler::Scheduler(size_t numThreads) : impl(make_unique<Impl>(numThreads)) {
   if (numThreads > maxThreadCount)
      runtime_error("too many threads");
}
//---------------------------------------------------------------------------
Scheduler::~Scheduler() noexcept = default;
//---------------------------------------------------------------------------
bool forceSingleThreaded = false;
//---------------------------------------------------------------------------
void Scheduler::run(std::string_view taskName, FunctionRef<void()> task) {
   if (!task)
      return;
   if (forceSingleThreaded) {
      task();
      return;
   }
   auto t = make_unique<Task>();
   t->taskId = ++impl->lastTaskId;
   t->func = task;
   t->workerCount = impl->threads.size();

   {
      for (auto& w : impl->threads) {
         //fmt::print("Notify available {}\n", w->threadId);
         w->task.store(t.get());
         w->workAvailable.notify_one();
      }

      for (auto& w : impl->threads) {
         for (size_t i = 0; w->task.load() && (i < spinCount); i++)
            hardwarePause();
         if (w->task.load()) {
            unique_lock ul(w->mut);
            w->workAvailable.notify_one();
         }
      }
   }

   for (size_t i = 0; (t->workerCount.load() >= 0) && (i < spinCount); i++)
      hardwarePause();
   {
      unique_lock ul(t->mut);
      if (t->workerCount.load() >= 0) {
         //fmt::print("Waiting for done\n");
         t->workDone.wait(ul, [&]() -> bool { return t->workerCount.load() < 0; });
      }
   }
}
//---------------------------------------------------------------------------
size_t Scheduler::getThreadId() noexcept {
   return localThreadId;
}
//---------------------------------------------------------------------------
size_t Scheduler::threadCount() {
   return impl->threads.size();
}
//---------------------------------------------------------------------------
size_t Scheduler::hardwareThreads() {
   return thread::hardware_concurrency();
}
//---------------------------------------------------------------------------
}