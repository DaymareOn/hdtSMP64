#pragma once
#ifdef CUDA

#include "hdtSkinnedMeshShape.h"
#include "hdtDispatcher.h"

namespace hdt
{
	class CudaBody
	{
		friend class CudaPerTriangleShape;
		friend class CudaPerVertexShape;
		friend class CudaInterface;
		friend class CudaMergeBuffer;
	public:
		CudaBody(SkinnedMeshBody* body);
		void synchronize();
		int deviceId();

	private:
		class Imp;
		std::shared_ptr<Imp> m_imp;
	};
	
	class CudaPerTriangleShape
	{
		template <typename T>
		friend class CudaCollisionPair;
		friend class CudaInterface;
	public:
		class Imp;

		CudaPerTriangleShape(PerTriangleShape* shape);
		void updateTree();
		int deviceId();

	private:
		std::shared_ptr<Imp> m_imp;
	};

	class CudaPerVertexShape
	{
		template <typename T>
		friend class CudaCollisionPair;
		friend class CudaInterface;
	public:
		class Imp;

		CudaPerVertexShape(PerVertexShape* shape);
		void updateTree();
		int deviceId();

	private:
		std::shared_ptr<Imp> m_imp;
	};

	class CudaMergeBuffer
	{
		template <typename T>
		friend class CudaCollisionPair;
	public:
		class Imp;

		CudaMergeBuffer(SkinnedMeshBody* body0, SkinnedMeshBody* body1);

		void launchTransfer();

		void apply(SkinnedMeshBody* body0, SkinnedMeshBody* body1, CollisionDispatcher* dispatcher);

	private:
		std::shared_ptr<Imp> m_imp;
	};

	template <typename T>
	class CudaCollisionPair
	{
	public:
		CudaCollisionPair(
			CudaPerVertexShape* shapeA,
			T* shapeB,
			int numCollisionPairs);

		void addPair(
			int offsetA,
			int offsetB,
			int sizeA,
			int sizeB,
			const Aabb& aabbA,
			const Aabb& aabbB);

		void launch(CudaMergeBuffer* merge, bool swap);

		int numPairs();

	private:
		class Imp;
		std::shared_ptr<Imp> m_imp;
	};

	class CudaInterface
	{
		struct CudaBuffers;

	public:
		static bool enableCuda;
		static int currentDevice;

		static CudaInterface* instance();

		bool hasCuda();

		void synchronize();

		void clearBufferPool();

		int deviceCount();

		void setCurrentDevice();

		static void launchInternalUpdate(
			std::shared_ptr<CudaBody> body,
			std::shared_ptr<CudaPerVertexShape> vertexShape,
			std::shared_ptr<CudaPerTriangleShape> triangleShape);

	private:

		CudaInterface();
		bool m_enabled;
	};
}
#endif