#pragma once
#include "IEventListener.h"

/*Plugins should register for messages from hdtSMP64 via SKSE during SKSE's PostLoad event.
* When ready, hdtSMP64 will send a message of type MSG_STARTUP containing a PluginInterface* as data.
* 
* A plugin MUST verify compatibility with the interface version before calling any other functions.
* The interface version is semantic, meaning that
*	- any incompatible API change increments the MAJOR version
*	- any backwards-compatible new feature increments the MINOR version
*	- a PATCH update does not change anything in this API
* 
* A plugin MUST verify compatibility with the Bullet version if it intends to interact with any Bullet object.
* Refer to Bullet documentation for information about their versioning scheme.
* 
* If compatible, the plugin may call other functions on the PluginInterface at any time.
* The PluginInterface pointer shall remain valid until shutdown.
*/

class btDynamicsWorld;

namespace hdt
{
	//Sent right before the physics simulation begins updating
	struct PreStepEvent
	{
		const btDynamicsWorld* world{ nullptr };
		float timeStep{ 0.0f };
	};

	//Sent right after the physics simulation has finished updating
	struct PostStepEvent
	{
		const btDynamicsWorld* world{ nullptr };
		float timeStep{ 0.0f };
	};

	using IPreStepListener = IEventListener<PreStepEvent>;
	using IPostStepListener = IEventListener<PostStepEvent>;

	class PluginInterface
	{
	public:
		enum MessageType : unsigned long
		{
			MSG_STARTUP,
		};

		struct Version
		{
			int major;
			int minor;
			int patch;
		};

		struct VersionInfo
		{
			Version interfaceVersion;
			Version bulletVersion;
		};

	public:
		//Consider the interface to be unstable for now
		constexpr static Version INTERFACE_VERSION{ 0, 1, 0 };
		
		//Is this defined somewhere already? Should it be?
		constexpr static Version BULLET_VERSION{ 3, 24, 0 };

	public:
		virtual ~PluginInterface() = default;

		virtual const VersionInfo& getVersionInfo() const = 0;

		virtual void addListener(IPreStepListener*) = 0;
		virtual void removeListener(IPreStepListener*) = 0;

		virtual void addListener(IPostStepListener*) = 0;
		virtual void removeListener(IPostStepListener*) = 0;
	};
}
