/*
Copyright 2021 Siyuan Pan <pansiyuan.cs@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/
#ifndef AJISAI_CORE_PARALLEL_H_
#define AJISAI_CORE_PARALLEL_H_

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>

#include "Ajisai/Math/Math.h"

#define API_EXPORT __attribute__((visibility("default")))

namespace Ajisai::Core {
// namespace Impl {
// struct ParallelContext {
//   uint32_t count = 0;
//   uint32_t chunkSize = 0;
//   std::atomic_uint32_t idx;
//   const std::function<void(uint32_t, uint32_t)>* func = nullptr;
//   constexpr ParallelContext() : idx(0) {}
//   ParallelContext(const ParallelContext& rhs)
//       : idx(rhs.idx.load()),
//         count(rhs.count),
//         chunkSize(rhs.chunkSize),
//         func(rhs.func) {}
//   bool Done() const { return idx >= count; }
// };

// class ParallelPool {
//  public:
//   ParallelPool() : stop{false}, nThreadRunning(0), nThreadFinish(0) {
//     auto n = std::thread::hardware_concurrency();
//     for (int id = 0; id != n; ++id) {
//       threads.emplace_back([=]() {
//         while (!stop) {
//           std::unique_lock<std::mutex> lk(workMutex);
//           while (works.empty() && !stop) {
//             workCv.wait(lk);
//           }

//           if (stop) return;

//           auto& work = works.front();
//           nThreadRunning++;
//           lk.unlock();
//           while (!work.Done()) {
//             auto begin = work.idx.fetch_add(work.chunkSize);
//             for (int i = begin; i < begin + work.chunkSize && i < work.count;
//                  ++i) {
//               (*work.func)(i, id);
//             }
//           }
//           lk.lock();
//           nThreadFinish++;
//           // finshCv.notify_all();

//           // while (nThreadRunning != nThreadFinish) {
//           //   finshCv.wait(lk);
//           // }

//           if (nThreadRunning != nThreadFinish) {
//             finshCv.wait(lk);
//           } else {
//             works.pop_front();
//             nThreadRunning = 0;
//             nThreadFinish = 0;
//             waitCv.notify_one();
//             finshCv.notify_all();
//           }

//           // if (!works.empty()) {
//           //   works.pop_front();
//           //   if (works.empty()) {
//           //     waitCv.notify_one();
//           //   }
//           // }
//           lk.unlock();
//         }
//       });
//     }
//   }

//   ~ParallelPool() {
//     stop = true;
//     workCv.notify_all();
//     for (auto& t : threads) {
//       t.join();
//     }
//   }

//   void Enqueue(const Impl::ParallelContext& ctx) {
//     std::lock_guard<std::mutex> lk(workMutex);
//     works.emplace_back(ctx);
//     workCv.notify_all();
//   }

//   void Wait() {
//     std::unique_lock<std::mutex> lk(workMutex);
//     while (!works.empty()) {
//       waitCv.wait(lk);
//     }
//   }

//  protected:
//   std::vector<std::thread> threads;
//   std::deque<Impl::ParallelContext> works;
//   std::mutex workMutex;
//   std::condition_variable workCv, waitCv, finshCv;
//   std::atomic_bool stop;
//   std::uint32_t nThreadRunning, nThreadFinish;
// };
// }  // namespace Impl

// static std::once_flag flag;
// static std::unique_ptr<Impl::ParallelPool> pool;

// inline void parallel_for(uint32_t count,
//                          const std::function<void(uint32_t, uint32_t)>& func,
//                          uint32_t chunkSize = 1) {
//   // static std::once_flag flag;
//   // static std::unique_ptr<Impl::ParallelPool> pool;
//   std::call_once(flag,
//                  [&]() { pool = std::make_unique<Impl::ParallelPool>(); });
//   Impl::ParallelContext ctx;
//   ctx.count = count;
//   ctx.chunkSize = chunkSize;
//   ctx.func = &func;
//   pool->Enqueue(ctx);
//   pool->Wait();
// }

// inline void thread_pool_finalize() { pool.reset(nullptr); }
void parallel_for(uint32_t count,
                  const std::function<void(uint32_t, uint32_t)>& func,
                  uint32_t chunkSize = 1);

inline void parallel_for_2D(
    const Math::Vector2i& dim,
    const std::function<void(Math::Vector2i, uint32_t)>& func,
    uint32_t chunkSize = 1) {
  parallel_for(
      dim.x() * dim.y(),
      [&](uint32_t idx, uint32_t tid) {
        auto x = idx % dim.x();
        auto y = idx / dim.x();
        func(Math::Vector2i(x, y), tid);
      },
      chunkSize);
}

void thread_pool_finalize();
}  // namespace Ajisai::Core

#endif