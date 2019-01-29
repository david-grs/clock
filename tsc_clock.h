#pragma once

#include <chrono>
#include <type_traits>
#include <thread>
#include <cstdint>
#include <stdexcept>

class TSCClock
{
public:
	static constexpr bool is_steady = true; // tsc is steady on all modern processors

	using time_point = uint64_t;
	static time_point now() noexcept;

	static void Initialise();

	static std::chrono::nanoseconds FromCycles(uint64_t);

	template <class DurationT>
	static uint64_t ToCycles(DurationT);

	static double GetFrequencyGHz() { return GetFrequencyGHzImpl(); }

private:
	static double& GetFrequencyGHzImpl()
	{
		static double TSCFreqGHz = .0;
		return TSCFreqGHz;
	}
};

namespace detail
{

inline void cpuid()
{
	uint64_t rax, rbx, rcx, rdx;
	__asm__ __volatile__("cpuid" : "=a"(rax), "=b"(rbx), "=d"(rdx), "=c"(rcx));
	asm volatile("" : : "r,m"(rax) : "memory");
}

inline uint64_t rdtsc()
{
	uint64_t rax, rdx;
	__asm__ __volatile__ ("rdtsc" : "=a"(rax), "=d"(rdx));
	return (rdx << 32) + rax;
}

inline uint64_t rdtscp()
{
	uint64_t rax, rdx;
	__asm__ __volatile__("rdtscp" : "=a"(rax), "=d"(rdx));
	return (rdx << 32) + rax;
}

inline uint64_t rdtscp(int& chip, int& core)
{
	uint64_t rax, rcx, rdx;
	__asm__ __volatile__("rdtscp" : "=a"(rax), "=d"(rdx), "=c"(rcx));
	chip = (rcx & 0xFFF000) >> 12;
	core = rcx & 0xFFF;
	return (rdx << 32) + rax;
}

inline double MeasureTSCFrequency()
{
	using SteadyClock = std::conditional_t<
			std::chrono::high_resolution_clock::is_steady,
			std::chrono::high_resolution_clock,
			std::chrono::steady_clock>;

	int chip, core, chip2, core2;

	const auto startTs = SteadyClock::now();
	asm volatile("" : : "r,m"(startTs) : "memory");

	const auto startTSC = detail::rdtscp(chip, core);
	detail::cpuid();

	std::this_thread::sleep_for(std::chrono::milliseconds(100));

	const auto endTSC = detail::rdtscp(chip2, core2);
	detail::cpuid();

	const auto endTs = SteadyClock::now();

	if (core != core2 || chip != chip2)
	{
		throw std::runtime_error("TSCClock: process needs to be set to a specific core");
	}

	const auto durationNs = std::chrono::duration_cast<std::chrono::nanoseconds>(endTs - startTs);
	return (endTSC - startTSC) / double(durationNs.count());
}

}

inline TSCClock::time_point TSCClock::now() noexcept
{
	return detail::rdtscp();
}

inline std::chrono::nanoseconds TSCClock::FromCycles(uint64_t cycles)
{
	const double nanoseconds{static_cast<double>(cycles) / GetFrequencyGHz()};
	return std::chrono::nanoseconds(static_cast<uint64_t>(nanoseconds));
}

template <class DurationT>
inline uint64_t TSCClock::ToCycles(DurationT duration)
{
	const double nanoseconds{static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count())};
	return static_cast<uint64_t>(nanoseconds * GetFrequencyGHz());
}

inline void TSCClock::Initialise()
{
	double& tscFreq = GetFrequencyGHzImpl();
	if (tscFreq != .0)
	{
		return;
	}

	double prevFreq = -1.0;
	for (int i = 0; i < 10; ++i)
	{
		tscFreq = detail::MeasureTSCFrequency();

		if (tscFreq > prevFreq * 0.9999 && tscFreq < prevFreq * 1.0001)
		{
			break;
		}

		prevFreq = tscFreq;
	}
}
