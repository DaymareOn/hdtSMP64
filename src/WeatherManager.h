#pragma once

#include <array>
#include <random>

namespace hdt
{
	class WeatherManager
	{
	public:
		static WeatherManager* get();

		WeatherManager(const WeatherManager&) = delete;
		WeatherManager& operator=(const WeatherManager&) = delete;

		static void runWeatherTick(float delta) { get()->runWeatherTickImpl(delta); }
		static RE::NiPoint3 getWindDirection() { return get()->m_precipDirection; }

	private:
		WeatherManager() = default;
		~WeatherManager() = default;

		static constexpr std::array NOT_EXTERIOR_WORLDS = {
			0x69857u, 0x1EE62u, 0x20DCBu, 0x1FAE2u, 0x34240u,
			0x50015u, 0x2C965u, 0x29AB7u, 0x4F838u, 0x3A9D6u,
			0x243DEu, 0xC97EBu, 0xC350Du, 0x1CDD3u, 0x1CDD9u,
			0x21EDBu, 0x1E49Du, 0x2B101u, 0x2A9D8u, 0x20BFEu
		};

		static constexpr float LONG_COOLDOWN = 5.0f;
		static constexpr float SHORT_COOLDOWN = 0.5f;

		RE::NiPoint3 m_precipDirection{ 0.f, 0.f, 0.f };
		float m_cooldown = 0.f;
		std::mt19937 m_rng{ std::random_device{}() };

		void runWeatherTickImpl(float delta);
		void clearWind(float cooldown);

		int randomGenerator(int min, int max);
		int randomGeneratorLowMoreProbable(int lowerMin, int lowerMax, int higherMin, int higherMax, int probability);
	};
}
