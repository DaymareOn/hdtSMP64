#pragma once

#include <memory>
#include <immintrin.h>

#include <tuple>
#include <string>

#ifndef __NVCC__
#define __host__
#define __device__
#define __forceinline__ __forceinline
#endif

#define CUDA_ERROR_CHECKING

#include "hdtCudaPlanarStruct.cuh"

namespace hdt
{
	class CudaPerVertexShape;
	class CudaPerTriangleShape;

	class cuResult
	{
	public:

#ifdef __NVCC__
#ifdef CUDA_ERROR_CHECKING
		cuResult(cudaError_t error = cudaGetLastError())
			: m_ok(error == cudaSuccess)
		{
			if (!m_ok)
			{
				m_message = cudaGetErrorString(error);
			}
		}
#else
		cuResult(cudaError_t = cudaSuccess)
			: m_ok(true) {}
#endif
#else
		bool check(std::string context)
		{
			if (!m_ok)
			{
				_MESSAGE("{}: {}", context.c_str(), m_message.c_str());
			}
			return m_ok;
		}
#endif

		operator bool()
		{
			return m_ok;
		}

		std::string message()
		{
			return m_message;
		}

	private:

		bool m_ok;
		std::string m_message;
	};

	// Internal penetration is the most common case where penetration is positive
	// No penetration is where it is zero, and the triangle is treated as unoriented
	// If penetration is negative, we handle it at the interface level by swapping two vertices and negating
	// penetration, making it equivalent to the internal case.
	enum cuPenetrationType
	{
		eInternal,
		eNone
	};

	struct cuVector3;

	union cuVector4
	{
		struct {
			float x;
			float y;
			float z;
			float w;
		};
		__m128 val;

#ifdef __NVCC__
		__device__ cuVector4();
		__device__ cuVector4(float ix, float iy, float iz, float iw);
		__device__ cuVector4(const cuVector3& v);
		__device__ explicit operator cuVector3() const;
		__device__ float* __restrict__ vals() { return &x; }
		__device__ const float* __restrict__ vals() const { return &x; }

		__device__ cuVector4 operator+(const cuVector4& o) const;
		__device__ cuVector4 operator-(const cuVector4& o) const;
		__device__ cuVector4 operator*(const float c) const;

		__device__ cuVector4& operator+=(const cuVector4& o);
		__device__ cuVector4& operator-=(const cuVector4& o);
		__device__ cuVector4& operator*=(const float c);

		__device__ float magnitude2() const;
		__device__ float magnitude() const;
		__device__ cuVector4 normalize() const;
#endif
	};

	struct cuVector3
	{
#ifdef __NVCC__
		__device__ cuVector3(float ix, float iy, float iz);
#endif
		float x;
		float y;
		float z;
	};

	struct cuTriangleIndices
	{
		int a;
		int b;
		int c;
	};

	struct cuPerVertexInput
	{
		int vertexIndex;
		float flexible;
	};

	struct cuPerTriangleInput
	{
		cuTriangleIndices vertexIndices;
		float flexible;
	};

	struct cuAabb
	{
#ifdef __NVCC__
		__device__ cuAabb();
#endif

		cuVector4 aabbMin;
		cuVector4 aabbMax;
	};

	struct cuBone
	{
		cuVector4 transform[4];
#ifdef __NVCC__
		__device__ const float* __restrict__ vals() const { return &transform[0].x; }
#endif
	};

	struct cuVertex
	{
		cuVector4 position;
		float weights[4];
		uint32_t bones[4];
	};

	// Never actually used, just need its size for setting results
	struct alignas(16) cuCollider
	{
		int vertexIndices[3];
		float flexible;
	};

	struct cuCollisionResult
	{
		cuVector4 posA;
		cuVector4 posB;
		cuVector4 normOnB;
		int colliderA;
		int colliderB;
		float depth;
	};

	struct cuCollisionSetup
	{
		int sizeA;
		int sizeB;
		int offsetA;
		int offsetB;
		cuAabb boundingBoxA;
		cuAabb boundingBoxB;
	};

	struct cuCollisionMerge
	{
		cuVector4 normal;
		cuVector4 posA;
		cuVector4 posB;
		float weight;
	};

	using VectorArray = ArrayType<cuVector3, float, float, float>;
	using BoundingBoxArray = ArrayType<cuAabb, VectorArray, VectorArray>;
	using VertexInputArray = ArrayType<cuPerVertexInput, int, float>;
	using TriangleInputArray = ArrayType<cuPerTriangleInput, ArrayType<cuTriangleIndices, int, int, int>, float, float>;

	// Data for calculating vertex positions and populating merge buffer
	struct cuBodyData
	{
		const cuVertex* __restrict__ vertexData;
		cuVector4* __restrict__ vertexBuffer;
		int numVertices;
	};

	// Data for populating merge buffer
	struct cuCollisionBodyData
	{
		const cuVertex* __restrict__ vertexData;
		cuVector4* __restrict__ vertexBuffer;
		const float* __restrict__ boneWeights;
		const int* __restrict__ boneMap;
	};

	// Data for per-collider calculations
	template<typename T>
	struct cuColliderData;

	struct VertexMargin
	{
		float margin;
	};

	struct TriangleMargin
	{
		float margin;
		float penetration;
	};

	template<>
	struct cuColliderData<CudaPerVertexShape>
	{
		const VertexInputArray input;
		BoundingBoxArray boundingBoxes;
		int numColliders;
		VertexMargin margin;
	};

	template<>
	struct cuColliderData<CudaPerTriangleShape>
	{
		const TriangleInputArray input;
		BoundingBoxArray boundingBoxes;
		int numColliders;
		TriangleMargin margin;
	};

	struct cuMergeBuffer
	{
		cuCollisionMerge* buffer;
		int x;
		int y;
		int dynx;
	};

	cuResult cuCreateStream(void** ptr);

	void cuDestroyStream(void* ptr);

	cuResult cuGetDeviceBuffer(void** buf, int size);

	cuResult cuGetHostBuffer(void** buf, int size);

	void cuFreeDevice(void* buf);

	void cuFreeHost(void* buf);

	cuResult cuCopyToDevice(void* dst, void* src, size_t n, void* stream);

	cuResult cuCopyToHost(void* dst, void* src, size_t n, void* stream);

	cuResult cuMemset(void* buf, int value, size_t n, void* stream);

	template <cuPenetrationType penType = eNone, typename T>
	cuResult cuRunCollision(
		void* stream,
		int n,
		bool swap,
		cuCollisionSetup* setup,
		cuColliderData<CudaPerVertexShape> inA,
		cuColliderData<T> inB,
		cuCollisionBodyData bodyA,
		cuCollisionBodyData bodyB,
		cuMergeBuffer mergeBuffer);

	cuResult cuInternalUpdate(
		void* stream,
		cuBodyData vertexData,
		const cuBone* boneData,
		cuColliderData<CudaPerVertexShape> perVertexData,
		int nVertexNodes,
		const std::pair<int, int>* vertexNodeData,
		cuAabb* vertexNodeOutput,
		cuColliderData<CudaPerTriangleShape> perTriangleData,
		int nTriangleNodes,
		const std::pair<int, int>* triangleNodeData,
		cuAabb* triangleNodeOutput);

	cuResult cuSynchronize(void* stream = nullptr);

	cuResult cuCreateEvent(void** ptr);

	void cuDestroyEvent(void* ptr);

	void cuRecordEvent(void* ptr, void* stream);

	void cuWaitEvent(void* ptr);

	void cuInitialize();

	int cuDeviceCount();

	void cuSetDevice(int id);

	int cuGetDevice();

	void* cuDevicePointer(void* ptr);
}