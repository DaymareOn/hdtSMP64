#include "WeatherManager.h"
#include "hdtSkyrimPhysicsWorld.h"

RE::NiPoint3 precipDirection { 0.f, 0.f, 0.f };

std::vector<uint32_t> notExteriorWorlds = 
{
	0x69857, 
	0x1EE62, 
	0x20DCB, 
	0x1FAE2, 
	0x34240, 
	0x50015, 
	0x2C965, 
	0x29AB7, 
	0x4F838, 
	0x3A9D6, 
	0x243DE, 
	0xC97EB, 
	0xC350D, 
	0x1CDD3, 
	0x1CDD9, 
	0x21EDB, 
	0x1E49D, 
	0x2B101, 
	0x2A9D8, 
	0x20BFE 
};

static inline size_t randomGeneratorLowMoreProbable(size_t lowermin, size_t lowermax, size_t highermin, size_t highermax, int probability) 
{
	std::mt19937 rng;
	rng.seed(std::random_device()());

	std::uniform_int_distribution<std::mt19937::result_type> dist(1, probability);

	if (dist(rng) == 1)
	{
		//higher
		rng.seed(std::random_device()());

		std::uniform_int_distribution<std::mt19937::result_type> distir(static_cast<uint32_t>(highermin), static_cast<uint32_t>(highermax));

		return distir(rng);
	}
	else
	{
		rng.seed(std::random_device()());

		std::uniform_int_distribution<std::mt19937::result_type> distir(static_cast<uint32_t>(lowermin), static_cast<uint32_t> (lowermax));

		return distir(rng);
	}
}

size_t hdt::randomGenerator(size_t min, size_t max) 
{
	std::mt19937 rng;
	rng.seed(std::random_device()());
	std::uniform_int_distribution<std::mt19937::result_type> dist(static_cast<uint32_t>(min), static_cast<uint32_t>(max));
	return dist(rng);
}

static inline RE::NiPoint3 crossProduct(RE::NiPoint3 A, RE::NiPoint3 B)
{
	return RE::NiPoint3(A.y * B.z - A.z * B.y, A.z * B.x - A.x * B.z, A.x * B.y - A.y * B.x);
}

// Calculates a dot product
static inline float dot(RE::NiPoint3 a, RE::NiPoint3 b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

// Calculates a cross product
static inline RE::NiPoint3 cross(RE::NiPoint3 a, RE::NiPoint3 b)
{
	return RE::NiPoint3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

static inline RE::NiPoint3 rotate(const RE::NiPoint3& v, const RE::NiPoint3& axis, float theta)
{
	const float cos_theta = cosf(theta);

	return (v * cos_theta) + (crossProduct(axis, v) * sinf(theta)) + (axis * dot(axis, v)) * (1 - cos_theta);
}

float hdt::magnitude(RE::NiPoint3 p)
{
	return sqrtf(p.x * p.x + p.y * p.y + p.z * p.z);
}

void hdt::WeatherCheck()
{
	RE::TESObjectCELL* cell = nullptr;
	RE::Actor* player = nullptr;

	const auto world = SkyrimPhysicsWorld::get();
	while (true)
	{
		const auto skyPtr = RE::Sky::GetSingleton();
		if (skyPtr)
		{
			player = RE::TESForm::LookupByID<RE::Actor>(0x14);
			if (!player || !player->loadedData)
			{
				// LOG("player null. Waiting for 5seconds");
				world->setWind(RE::NiPoint3{ 0,0,0 }, 0.f, 1); // remove wind immediately
				Sleep(5000);
				continue;
			}

			cell = player->parentCell;
			if (!cell)
			{
				world->setWind(RE::NiPoint3{ 0,0,0 }, 0.0f, 1); // remove wind immediately
				continue;
			}

	#ifdef SKYRIMVR
			RE::TESWorldSpace* worldSpace = cell->GetRuntimeData().unk120;
	#else
			RE::TESWorldSpace* worldSpace = cell->GetRuntimeData().worldSpace;
	#endif
			if (!worldSpace) // Interior cell
			{
				//LOG("In interior cell. Waiting for 5 seconds");
				world->setWind(RE::NiPoint3{ 0,0,0 }, 0, 1); // remove wind immediately
				Sleep(5000);
				continue;
			}
			else
			{
				if (std::find(notExteriorWorlds.begin(), notExteriorWorlds.end(), worldSpace->formID) != notExteriorWorlds.end())
				{
					//LOG("In interior cell world. Waiting for 5 seconds");
					world->setWind(RE::NiPoint3{ 0,0,0 }, 0, 1); // remove wind immediately
					Sleep(5000);
					continue;
				}
			}

				// Wind Detection
				const float range = (randomGeneratorLowMoreProbable(0, 5, 6, 50, 10) / 10.0f);
				precipDirection = RE::NiPoint3 { 0.f, 1.f, 0.f };
				if (skyPtr->currentWeather)
				{
					logger::info
					(
						"Wind Speed: {:.2f}, Wind Direction: {:.2f}, Weather Wind Speed: {:.2f} WindDir: {:.2f} WindDirRange: {:.2f}", 
						skyPtr->windSpeed, 
						skyPtr->windAngle, 
						static_cast<float>(skyPtr->currentWeather->data.windSpeed), 
						static_cast<float>(skyPtr->currentWeather->data.windDirection * 180.0f / 256.0f), 
						static_cast<float>(skyPtr->currentWeather->data.windDirectionRange * 360.0f / 256.0f)
					);
					
					// use weather wind info
					// Wind Speed is the only thing that changes. Wind direction and range are same all the time as set in CK.
					const float theta = (((skyPtr->currentWeather->data.windDirection) * 180.0f) / 256.0f) - 90.f + randomGenerator(static_cast<size_t>(-range), static_cast<size_t>(range));
					precipDirection = rotate(precipDirection, RE::NiPoint3(0, 0, 1.0f), theta / 57.295776f);
					world->setWind(precipDirection, world->m_windStrength * scaleSkyrim * skyPtr->windSpeed);
				}
				else 
				{
					logger::info("Wind Speed: {:.2f}, Wind Direction: {:.2f}", skyPtr->windSpeed, skyPtr->windAngle);
					
					// use sky wind info
					const float theta = (((skyPtr->windAngle) * 180.0f) / 256.0f) - 90.f + (randomGenerator(0, 2 * static_cast<size_t>(range)) - range);
					precipDirection = rotate(precipDirection, RE::NiPoint3(0, 0, 1.0f), theta / 57.295776f);
					world->setWind(precipDirection, world->m_windStrength * scaleSkyrim * skyPtr->windSpeed);
				}
				Sleep(500);
		}
		else
		{
			world->setWind(RE::NiPoint3{ 0,0,0 }, 0, 1); // remove wind immediately
			//LOG("Sky is null. waiting for 5 seconds.");
			Sleep(5000);
		}
	}
}

RE::NiPoint3* hdt::getWindDirection()
{
	return &precipDirection;
}
