#pragma once

#include <algorithm>
#include <future>
#include <thread>
#include <vector>

namespace hdt
{
	// Distribute [0, n) across std::thread::hardware_concurrency() async tasks.
	// chunkFn(begin, end) is called once per chunk. Blocks until all complete.
	template <typename F>
	inline void ParallelForChunks(size_t n, F chunkFn)
	{
		if (n == 0)
			return;
		const unsigned nThreads = std::max(1u, std::thread::hardware_concurrency());
		const size_t chunkSize = (n + nThreads - 1) / nThreads;
		std::vector<std::future<void>> futures;
		futures.reserve(nThreads);
		for (size_t i = 0; i < n; i += chunkSize) {
			size_t end = std::min(i + chunkSize, n);
			futures.push_back(std::async(std::launch::async, [chunkFn, i, end]() {
				chunkFn(i, end);
			}));
		}
		for (auto& f : futures)
			f.get();
	}

}  // namespace hdt
