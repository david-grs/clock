#include <iostream>
#include <vector>

#include "tsc_clock.h"

#include <x86intrin.h>

int main()
{
	TSCClock::Initialise();
	std::cout << TSCClock::GetFrequencyGHz() << std::endl;

	const auto start = TSCClock::now();
	std::vector<int> v(1024*1024, 123);
	const auto end = TSCClock::now();

	std::cout << TSCClock::FromCycles(end - start).count() << "ns" << std::endl;
	std::cout << v.size() << std::endl;
	return 0;
}
