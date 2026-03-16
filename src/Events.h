#pragma once

namespace Events
{
	struct FrameEvent
	{
		bool gamePaused;
	};

	struct FrameSyncEvent
	{
	};

	struct ShutdownEvent
	{
	};

	struct SkinAllHeadGeometryEvent
	{
		RE::NiNode* skeleton = nullptr;
		RE::BSFaceGenNiNode* headNode = nullptr;
		bool hasSkinned = false;
	};

	struct SkinSingleHeadGeometryEvent
	{
		RE::NiNode* skeleton = nullptr;
		RE::BSFaceGenNiNode* headNode = nullptr;
		RE::BSGeometry* geometry = nullptr;
	};

	struct ArmorAttachEvent
	{
		RE::NiNode* armorModel = nullptr;
		RE::NiNode* skeleton = nullptr;
		RE::NiAVObject* attachedNode = nullptr;
		bool hasAttached = false;
	};

	struct ArmorDetachEvent
	{
		RE::Actor* actor = nullptr;
		bool hasDetached = false;
	};

	namespace Sources
	{
		class FrameEventSource : public RE::BSTEventSource<FrameEvent>
		{
		public:
			static FrameEventSource* GetSingleton();
		};

		class FrameSyncEventSource : public RE::BSTEventSource<FrameSyncEvent>
		{
		public:
			static FrameSyncEventSource* GetSingleton();
		};

		class ShutdownEventEventSource : public RE::BSTEventSource<ShutdownEvent>
		{
		public:
			static ShutdownEventEventSource* GetSingleton();
		};

		class SkinAllHeadGeometryEventSource : public RE::BSTEventSource<SkinAllHeadGeometryEvent>
		{
		public:
			static SkinAllHeadGeometryEventSource* GetSingleton();
		};

		class SkinSingleHeadGeometryEventSource : public RE::BSTEventSource<SkinSingleHeadGeometryEvent>
		{
		public:
			static SkinSingleHeadGeometryEventSource* GetSingleton();
		};

		class ArmorAttachEventSource : public RE::BSTEventSource<ArmorAttachEvent>
		{
		public:
			static ArmorAttachEventSource* GetSingleton();
		};

		class ArmorDetachEventSource : public RE::BSTEventSource<ArmorDetachEvent>
		{
		public:
			static ArmorDetachEventSource* GetSingleton();
		};
	}

	namespace Sinks
	{
		class FreezeEventHandler : public RE::BSTEventSink<RE::MenuOpenCloseEvent>
		{
		public:
			static FreezeEventHandler* GetSingleton();
			static void Register();
			static void Unregister();

		public:
			RE::BSEventNotifyControl ProcessEvent(const RE::MenuOpenCloseEvent* a_event, RE::BSTEventSource<RE::MenuOpenCloseEvent>* a_eventSource) override;

		private:
		};
	}

	//
	void Register();
	void Unregister();
}
