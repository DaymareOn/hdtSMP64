#pragma once

#include "hdtDispatcher.h"
#include <BulletCollision/CollisionDispatch/btSimulationIslandManager.h>

namespace hdt
{
	class SimulationIslandManager : public btSimulationIslandManager
	{
	public:
		void updateActivationState(btCollisionWorld* colWorld, btDispatcher* dispatcher) override;

		void findUnions(btDispatcher* dispatcher, btCollisionWorld* colWorld);
		//void buildAndProcessIslands(btDispatcher* dispatcher, btCollisionWorld* collisionWorld, IslandCallback* callback);
	};
}
