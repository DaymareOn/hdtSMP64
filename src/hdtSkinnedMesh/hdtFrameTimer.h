#pragma once

namespace hdt
{
	class FrameTimer
	{
	public:

		static FrameTimer* instance();

		enum Events
		{
			e_Start,
			e_Internal,
			e_Launched,
			e_End
		};

		enum Measurements
		{
			e_InternalUpdate,
			e_CollisionLaunch,
			e_CollisionProcess,
			e_Total
		};

		void reset(int nFrames);
		void addManifoldCount(int nManifolds);
		void logEvent(Events e);

		bool running();
		bool cudaFrame();

	private:

		int m_nFrames;
		std::map<Events, int64_t> m_timings;

		int m_count;
		std::map<Measurements, float> m_sumsCPU;
		std::map<Measurements, float> m_sumsSquaredCPU;
		std::map<Measurements, float> m_sumsGPU;
		std::map<Measurements, float> m_sumsSquaredGPU;

		int m_nManifoldsCPU;
		int m_nManifolds2CPU;
		int m_nManifoldsGPU;
		int m_nManifolds2GPU;
	};
}
