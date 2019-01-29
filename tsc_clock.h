#pragma once

#include <ctime>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <cstring>

class TSCClock
{
public:
	static void Initialise();

	static uint64_t Now();
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

inline uint64_t rdtscp(int& chip, int& core)
{
	uint64_t rax, rcx, rdx;
	__asm__ __volatile__("rdtscp" : "=a"(rax), "=d"(rdx), "=c"(rcx));
	chip = (rcx & 0xFFF000) >> 12;
	core = rcx & 0xFFF;
	return (rdx << 32) + rax;
}

inline bool MeasureTSCFrequency(double& freq)
{
	static const std::size_t SleepDurationNs = 20000000;

	struct timespec ts;
	ts.tv_nsec = SleepDurationNs;
	ts.tv_sec = 0;

	int chip, core, chip2, core2;

	detail::cpuid();
	const auto startTs = detail::rdtscp(chip, core);

	const int ret = nanosleep(&ts, &ts);

	const auto endTs = detail::rdtscp(chip2, core2);
	detail::cpuid();

	if (core != core2 || chip != chip2)
	{
		throw std::runtime_error("tsc_clock: process needs to be pin to a specific core while calibrating tsc");
	}

	if (ret != 0 && errno != EINTR)
	{
		throw std::runtime_error(std::string("TSCClock: nanosleep failed, reason: ") + std::strerror(errno));
	}
	else if (ret != 0)
	{
		return false;
	}

	freq = (endTs - startTs) / double(SleepDurationNs);
	return true;
}

}

inline uint64_t TSCClock::Now()
{
	return detail::rdtsc();
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
	int i;
	for (i = 0; i < 1000000; ++i)
	{
		while (!detail::MeasureTSCFrequency(tscFreq));

		if (tscFreq > prevFreq * 0.9999 && tscFreq < prevFreq * 1.0001)
		{
			return;
		}

		prevFreq = tscFreq;
	}
}


