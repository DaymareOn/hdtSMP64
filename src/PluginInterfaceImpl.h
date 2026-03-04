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

		void onPostPostLoad();

		void onPreStep(const PreStepEvent& e) { m_preStepDispatcher.SendEvent(std::addressof(e)); }
		void onPostStep(const PostStepEvent& e) { m_postStepDispatcher.SendEvent(std::addressof(e)); }

		void init(const SKSE::LoadInterface* skse);

	private:
		VersionInfo m_versionInfo{ INTERFACE_VERSION, BULLET_VERSION };
		RE::BSTEventSource<PreStepEvent> m_preStepDispatcher;
		RE::BSTEventSource<PostStepEvent> m_postStepDispatcher;

		SKSE::PluginHandle m_sksePluginHandle;
		SKSE::MessagingInterface* m_skseMessagingInterface;
	};

	extern PluginInterfaceImpl g_pluginInterface;
}
