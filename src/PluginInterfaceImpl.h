#pragma once

#include "PluginAPI.h"

namespace hdt
{
	class PluginInterfaceImpl final : public PluginInterface
	{
	public:
		PluginInterfaceImpl() = default;
		~PluginInterfaceImpl() = default;

		PluginInterfaceImpl(const PluginInterfaceImpl&) = delete;
		PluginInterfaceImpl& operator=(const PluginInterfaceImpl&) = delete;

		virtual const VersionInfo& getVersionInfo() const { return m_versionInfo; }

		virtual void addListener(IPreStepListener* l) override;
		virtual void removeListener(IPreStepListener* l) override;

		virtual void addListener(IPostStepListener* l) override;
		virtual void removeListener(IPostStepListener* l) override;

		virtual void addMeshOwner(IMeshOwner* owner, const MeshOwnerInfo& info) override;
		virtual void removeMeshOwner(IMeshOwner* owner) override;

		virtual void reloadActorPhysics(RE::Actor* actor) override;

		virtual void addFrameListener(IFrameListener* l) override;
		virtual void removeFrameListener(IFrameListener* l) override;

		virtual void addFrameSyncListener(IFrameSyncListener* l) override;
		virtual void removeFrameSyncListener(IFrameSyncListener* l) override;

		virtual void addShutdownListener(IShutdownListener* l) override;
		virtual void removeShutdownListener(IShutdownListener* l) override;

		void onPostPostLoad();

		void onPreStep(const PreStepEvent& e) { m_preStepDispatcher.SendEvent(std::addressof(e)); }
		void onFrame(const PluginFrameEvent& e) { m_frameDispatcher.SendEvent(std::addressof(e)); }
		void onFrameSync()
		{
			const PluginFrameSyncEvent e;
			m_frameSyncDispatcher.SendEvent(std::addressof(e));
		}
		void onShutdown()
		{
			const PluginShutdownEvent e;
			m_shutdownDispatcher.SendEvent(std::addressof(e));
		}
		void onPostStep(const PostStepEvent& e) { m_postStepDispatcher.SendEvent(std::addressof(e)); }

		void init(const SKSE::LoadInterface* skse);

	private:
		VersionInfo m_versionInfo{ INTERFACE_VERSION, BULLET_VERSION };
		RE::BSTEventSource<PreStepEvent> m_preStepDispatcher;
		RE::BSTEventSource<PostStepEvent> m_postStepDispatcher;
		RE::BSTEventSource<PluginFrameEvent> m_frameDispatcher;
		RE::BSTEventSource<PluginFrameSyncEvent> m_frameSyncDispatcher;
		RE::BSTEventSource<PluginShutdownEvent> m_shutdownDispatcher;

		SKSE::PluginHandle m_sksePluginHandle;
		SKSE::MessagingInterface* m_skseMessagingInterface;
	};

	extern PluginInterfaceImpl g_pluginInterface;
}
