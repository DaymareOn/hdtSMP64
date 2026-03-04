#pragma once

namespace hdt 
{
	float magnitude(RE::NiPoint3 p);
	size_t randomGenerator(size_t min, size_t max);
	void WeatherCheck();
	RE::NiPoint3* getWindDirection();
}
