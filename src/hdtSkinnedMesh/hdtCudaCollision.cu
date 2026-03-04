#include "hdtCudaCollision.cuh"

#include "math.h"

// Check collider bounding boxes on the GPU. This reduces the total amount of work, but is bad for
// divergence and increases register usage. Probably no longer very useful with vertex lists working.
//#define GPU_BOUNDING_BOX_CHECK

namespace hdt
{
    // Block size for map type operations (vertex and bounding box calculations). There's no inter-warp
    // reduction in these, so the value isn't very important.
    __host__ __device__ constexpr int cuMapBlockSize() { return 128; }

    // Block size for bounding box reduction. This needs 8 threads per box, so the default 256 will work in
    // chunks of 32 boxes.
    __host__ __device__ constexpr int cuReduceBlockSize() { return 256; }

    template<typename T>
    constexpr int collisionBlockSize();

    // Block size for collision checking. Must be a power of 2 for the simple inter-warp reductions to work,
    // and at least 64 for the merge buffer updates.
    template<>
    constexpr int collisionBlockSize<CudaPerVertexShape>() { return 256; }
    template<>
    constexpr int collisionBlockSize<CudaPerTriangleShape>() { return 256; }

    // Maximum number of vertices per patch
    __host__ __device__
    constexpr int vertexListSize() { return 256; }

    // Maximum number of iterations of collision checking with a single vertex list. If there are too many
    // potential collisions to finish in this number of passes, we compute the second vertex list as well.
    __device__
    constexpr int vertexListThresholdFactor() { return 4; }

    __device__ cuVector4::cuVector4()
    {}

    __device__ __forceinline__ cuVector4::cuVector4(float ix, float iy, float iz, float iw)
        : x(ix), y(iy), z(iz), w(iw)
    {}

    __device__ __forceinline__ cuVector4::cuVector4(const cuVector3& v)
        : x(v.x), y(v.y), z(v.z)
    {}

    __device__ __forceinline__ cuVector4::operator cuVector3() const
    {
        return { x, y, z };
    }

    __device__ __forceinline__ cuVector4 cuVector4::operator+(const cuVector4& o) const
    {
        return { x + o.x, y + o.y, z + o.z, w + o.w };
    }

    __device__ __forceinline__ cuVector4 cuVector4::operator-(const cuVector4& o) const
    {
        return { x - o.x, y - o.y, z - o.z, w - o.w };
    }

    __device__ __forceinline__ cuVector4 cuVector4::operator*(const float c) const
    {
        return { x * c, y * c, z * c, w * c };
    }

    __device__ __forceinline__ cuVector4& cuVector4::operator+=(const cuVector4& o)
    {
        *this = *this + o;
        return *this;
    }

    __device__ __forceinline__ cuVector4& cuVector4::operator-=(const cuVector4& o)
    {
        *this = *this - o;
        return *this;
    }

    __device__ __forceinline__ cuVector4& cuVector4::operator *= (const float c)
    {
        *this = *this * c;
        return *this;
    }

    __device__
        cuVector4 crossProduct(const cuVector4& v1, const cuVector4& v2)
    {
        return { v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x, 0 };
    }

    __device__
        float dotProduct(const cuVector4& v1, const cuVector4& v2)
    {
        return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
    }

    __device__ float cuVector4::magnitude2() const
    {
        return dotProduct(*this, *this);
    }

    __device__ float cuVector4::magnitude() const
    {
        return sqrt(magnitude2());
    }

    __device__ cuVector4 cuVector4::normalize() const
    {
        return *this * rsqrt(magnitude2());
    }

    __device__ cuVector3::cuVector3(float ix, float iy, float iz)
        : x(ix), y(iy), z(iz)
    {}

    __device__ __forceinline__ cuVector4 perElementMin(const cuVector4& v1, const cuVector4& v2)
    {
        return { min(v1.x, v2.x), min(v1.y, v2.y), min(v1.z, v2.z), min(v1.w, v2.w) };
    }

    __device__ __forceinline__ cuVector4 perElementMax(const cuVector4& v1, const cuVector4& v2)
    {
        return { max(v1.x, v2.x), max(v1.y, v2.y), max(v1.z, v2.z), max(v1.w, v2.w) };
    }

    __device__ cuAabb::cuAabb()
        : aabbMin({ FLT_MAX, FLT_MAX, FLT_MAX }), aabbMax({ -FLT_MAX, -FLT_MAX, -FLT_MAX })
    {}

    __device__
        bool boundingBoxCollision(const cuAabb& b1, const cuAabb b2)
    {
        return !(b1.aabbMin.x > b2.aabbMax.x ||
            b1.aabbMin.y > b2.aabbMax.y ||
            b1.aabbMin.z > b2.aabbMax.z ||
            b1.aabbMax.x < b2.aabbMin.x ||
            b1.aabbMax.y < b2.aabbMin.y ||
            b1.aabbMax.z < b2.aabbMin.z);
    }

    template <unsigned int BlockSize = cuMapBlockSize()>
    __global__ void kernelPerVertexUpdate(
        cuColliderData<CudaPerVertexShape> colliders,
        const cuVector4* __restrict__ vertexData)
    {
        int index = (blockIdx.x * BlockSize + threadIdx.x) >> 2;
        int stride = (BlockSize * gridDim.x) >> 2;
        int startIndex = index & ~7;

        int tid = threadIdx.x;
        int threadInWarp = tid & 0x1f;
        int quarterWarp = threadInWarp >> 3;
        int eighthWarp = threadInWarp >> 2;
        int threadInEighthWarp = tid & 0x03;
        int sourceThread = ((threadInWarp >> 1) & ~0x03) | threadInEighthWarp;

        float margin = colliders.margin.margin;

        for (int i = startIndex; i < colliders.numColliders; i += stride)
        {
            float v;
            if (i + eighthWarp < colliders.numColliders)
            {
                // Vertex index access is across 64 bytes, and we'll almost certainly want to read the next
                // 64, so cache it.
                const cuPerVertexInput& data = colliders.input[i + eighthWarp];
                v = vertexData[__ldg(&data.vertexIndex)].vals()[threadInEighthWarp];
            }

            float m = margin * __shfl_sync(0xffffffff, v, threadInWarp | 0x03);
            float vmin = v - m;
            float vmax = v + m;

            float v0 = __shfl_sync(0xffffffff, (tid & 0x10) ? vmax : vmin, sourceThread | ((tid & 4) << 2));
            float v1 = __shfl_sync(0xffffffff, (tid & 0x10) ? vmin : vmax, sourceThread | ((~tid & 4) << 2));
            if (i + quarterWarp < colliders.numColliders)
            {
                colliders.boundingBoxes[i].aabbMin.vals()[threadInWarp] = (threadInWarp & 4) ? v1 : v0;
                if (i + quarterWarp + 4 < colliders.numColliders)
                {
                    colliders.boundingBoxes[i + 4].aabbMin.vals()[threadInWarp] = (threadInWarp & 4) ? v0 : v1;
                }
            }
        }
    }

    template <unsigned int BlockSize = cuMapBlockSize()>
    __global__ void kernelPerTriangleUpdate(
        cuColliderData<CudaPerTriangleShape> colliders,
        const cuVector4* __restrict__ vertexData)
    {
        int index = (blockIdx.x * BlockSize + threadIdx.x) >> 2;
        int stride = (BlockSize * gridDim.x) >> 2;
        int startIndex = index & ~7;

        int tid = threadIdx.x;
        int threadInWarp = tid & 0x1f;
        int quarterWarp = threadInWarp >> 3;
        int eighthWarp = threadInWarp >> 2;
        int threadInEighthWarp = tid & 0x03;
        int sourceThread = ((threadInWarp >> 1) & ~0x03) | threadInEighthWarp;

        float penetration = abs(colliders.margin.penetration);
        float margin = colliders.margin.margin / 3;

        for (int i = startIndex; i < colliders.numColliders; i += stride)
        {
            int index;
            float v;
            float vmin, vmax, sum;
            if (i + eighthWarp < colliders.numColliders)
            {
                index = reinterpret_cast<const int*>(&colliders.input[i])[threadInWarp];
            }
            int ia = __shfl_sync(0xffffffff, index, threadInWarp & ~3);
            int ib = __shfl_sync(0xffffffff, index, (threadInWarp & ~3) | 1);
            int ic = __shfl_sync(0xffffffff, index, (threadInWarp & ~3) | 2);
            if (i + eighthWarp < colliders.numColliders)
            {
                // Triangle index access coalesces fairly well (it's all in the same 128-byte block) but we
                // currently do 3 accesses to it, so caching will help.
                v = vertexData[ia].vals()[threadInEighthWarp];
                vmin = vmax = sum = v;
                v = vertexData[ib].vals()[threadInEighthWarp];
                vmin = min(v, vmin);
                vmax = max(v, vmax);
                sum += v;
                v = vertexData[ic].vals()[threadInEighthWarp];
                vmin = min(v, vmin);
                vmax = max(v, vmax);
                sum += v;
            }

            float m = max(margin * __shfl_sync(0xffffffff, sum, threadInWarp | 0x03), penetration);

            vmin -= m;
            vmax += m;

            float v0 = __shfl_sync(0xffffffff, (tid & 0x10) ? vmax : vmin, sourceThread | ((tid & 4) << 2));
            float v1 = __shfl_sync(0xffffffff, (tid & 0x10) ? vmin : vmax, sourceThread | ((~tid & 4) << 2));
            if (i + quarterWarp < colliders.numColliders)
            {
                colliders.boundingBoxes[i].aabbMin.vals()[threadInWarp] = (threadInWarp & 4) ? v1 : v0;
                if (i + quarterWarp + 4 < colliders.numColliders)
                {
                    colliders.boundingBoxes[i + 4].aabbMin.vals()[threadInWarp] = (threadInWarp & 4) ? v0 : v1;
                }
            }
        }
    }

    template <unsigned int BlockSize = cuReduceBlockSize()>
    __global__ void kernelBoundingBoxReduce(
        int n,
        const std::pair<int, int>* __restrict__ nodeData,
        const BoundingBoxArray boundingBoxes,
        cuAabb* __restrict__ output)
    {
        __shared__ float shared[BlockSize >> 1];

        int tid = threadIdx.x;
        int threadInWarp = tid & 0x1f;
        
        for (int block = blockIdx.x; block < n; block += gridDim.x)
        {
            int firstBox = nodeData[block].first;
            int aabbCount = nodeData[block].second << 3;

            float v = (threadInWarp < aabbCount) ? reinterpret_cast<const float*>(&boundingBoxes[firstBox])[threadInWarp] : (threadInWarp & 4) ? -FLT_MAX : FLT_MAX;
            for (int i = threadInWarp + BlockSize; i < aabbCount; i += BlockSize)
            {
                float temp = reinterpret_cast<const float*>(&boundingBoxes[firstBox])[i];
                v = (threadInWarp & 4) ? max(temp, v) : min(temp, v);
            }
            for (int i = BlockSize; i > 32;)
            {
                int j = i >> 1;

                if (tid < i && tid >= j)
                {
                    shared[tid - j] = v;
                }
                __syncthreads();
                if (tid < j)
                {
                    float temp = shared[tid];
                    v = (threadInWarp & 4) ? max(temp, v) : min(temp, v);
                }
                i = j;
            }
            float temp = __shfl_xor_sync(0xffffffff, v, 16);
            v = (threadInWarp & 4) ? max(temp, v) : min(temp, v);
            temp = __shfl_xor_sync(0xffffffff, v, 8);
            v = (threadInWarp & 4) ? max(temp, v) : min(temp, v);

            if (tid < 8)
            {
                reinterpret_cast<float*>(&output[block])[threadInWarp] = v;
            }
        }
    }

    template <unsigned int BlockSize = cuMapBlockSize()>
    __global__ void kernelBodyUpdate(
        cuBodyData data,
        const cuBone* __restrict__ boneData)
    {
        // We work with an entire warp per vertex here
        int index = (blockIdx.x * BlockSize + threadIdx.x) >> 5;
        int stride = (BlockSize * gridDim.x) >> 5;

        // But we do 8 vertices sequentially, so we can write a full 32 values back at the end
        index <<= 3;
        stride <<= 3;

        int tid = threadIdx.x;
        int threadInWarp = tid & 0x1f;
        int threadInHalfWarp = tid & 0x0f;
        int halfWarp = threadInWarp >> 4;
        int eighthWarp = threadInWarp >> 2;
        int element = eighthWarp & 0x03;

        for (int i = index; i < data.numVertices; i += stride)
        {
            float v;
            float result;
            for (int j = 0; j < 8 && i + j < data.numVertices; ++j)
            {
                v = boneData[data.vertexData[i + j].bones[halfWarp]].vals()[threadInHalfWarp] * data.vertexData[i + j].position.vals()[element] * data.vertexData[i + j].weights[halfWarp];
                v += boneData[data.vertexData[i + j].bones[halfWarp + 2]].vals()[threadInHalfWarp] * data.vertexData[i + j].position.vals()[element] * data.vertexData[i + j].weights[halfWarp + 2];
                v += __shfl_xor_sync(0xffffffff, v, 4);
                v += __shfl_xor_sync(0xffffffff, v, 8);
                v += __shfl_xor_sync(0xffffffff, v, 16);
                if (eighthWarp == j)
                {
                    result = v;
                }
            }
            if (i + eighthWarp < data.numVertices)
            {
                // Note we exploit the fact that the output values are contiguous here to (hopefully) do a
                // full 128-byte transaction.
                data.vertexBuffer[i].vals()[threadInWarp] = result;
            }
        }
    }

    // collidePair does the actual collision between two colliders, always a vertex and some other type. It
    // should modify output if and only if there is a collision.
    template <cuPenetrationType>
    __device__ bool collidePair(
        const cuPerVertexInput& __restrict__ inputA,
        const cuPerVertexInput& __restrict__ inputB,
        const cuVector4* __restrict__ vertexDataA,
        const cuVector4* __restrict__ vertexDataB,
        const VertexMargin marginA,
        const VertexMargin marginB,
        cuCollisionResult& output)
    {
        const cuVector4 vA = vertexDataA[inputA.vertexIndex];
        const cuVector4 vB = vertexDataB[inputB.vertexIndex];

        float rA = vA.w * marginA.margin;
        float rB = vB.w * marginB.margin;
        float bound2 = (rA + rB) * (rA + rB);
        cuVector4 diff = vA - vB;
        float dist2 = diff.magnitude2();
        float len = sqrt(dist2);
        float dist = len - (rA + rB);
        if (dist2 <= bound2 && (dist < output.depth))
        {
            if (len <= FLT_EPSILON)
            {
                diff = { 1, 0, 0, 0 };
            }
            else
            {
                diff = diff.normalize();
            }
            output.depth = dist;
            output.normOnB = diff;
            output.posA = vA - diff * rA;
            output.posB = vB + diff * rB;
            return true;
        }
        return false;
    }

    template <cuPenetrationType penType>
    __device__ bool collidePair(
        const cuPerVertexInput& __restrict__ inputA,
        const cuPerTriangleInput& __restrict__ inputB,
        const cuVector4* __restrict__ vertexDataA,
        const cuVector4* __restrict__ vertexDataB,
        const VertexMargin marginA,
        const TriangleMargin marginB,
        cuCollisionResult& output)
    {
        cuVector4 s = vertexDataA[inputA.vertexIndex];
        float r = s.w * marginA.margin;
        cuVector4 p0 = vertexDataB[inputB.vertexIndices.a];
        cuVector4 p1 = vertexDataB[inputB.vertexIndices.b];
        cuVector4 p2 = vertexDataB[inputB.vertexIndices.c];
        float margin = (p0.w + p1.w + p2.w) / 3.0;
        float penetration = marginB.penetration * margin;
        margin *= marginB.margin;

        // Compute unit normal and twice area of triangle
        cuVector4 ab = p1 - p0;
        cuVector4 ac = p2 - p0;
        cuVector4 raw_normal = crossProduct(ab, ac);
        float area2 = raw_normal.magnitude2();
        float area = sqrt(area2);

        // Check for degenerate triangles
        if (area < FLT_EPSILON)
        {
            return false;
        }
        cuVector4 normal = raw_normal * (1.0 / area);

        // Compute distance from point to plane and its projection onto the plane
        cuVector4 ap = s - p0;
        float distance = dotProduct(ap, normal);

        float radiusWithMargin = r + margin;
        if (penType == eNone)
        {
            // Two-sided check: make sure distance is positive and normal is in the correct direction
            if (distance < 0)
            {
                distance = -distance;
                normal *= -1.0;
            }
        }
        else if (distance < penetration)
        {
            // One-sided check: make sure sphere center isn't too far on the wrong side of the triangle
            return false;
        }

        // Don't bother to do any more if there's no collision or we already have a deeper one
        float depth = distance - radiusWithMargin;
        if (depth >= -FLT_EPSILON || depth >= output.depth)
        {
            return false;
        }

        // Compute triple products and check the projection lies in the triangle
        cuVector4 bp = s - p1;
        cuVector4 cp = s - p2;
        ac = crossProduct(ap, bp);
        ab = crossProduct(cp, ap);
        float areaC = dotProduct(ac, raw_normal);
        float areaB = dotProduct(ab, raw_normal);
        float areaA = area2 - areaB - areaC;
        if (areaA < 0 || areaB < 0 || areaC < 0)
        {
            return false;
        }

        output.normOnB = normal;
        output.posA = s - normal * r;
        output.posB = s - normal * (distance - margin);
        output.depth = depth;
        return true;
    }

    template<int BlockSize>
    __device__ int kernelComputeVertexList(
        int start,
        int n,
        int tid,
        const BoundingBoxArray boundingBoxes,
        const cuAabb& boundingBox,
        int* intShared,
        int* vertexList
    )
    {
        int* partialSums = intShared + 32;
        int threadInWarp = tid & 0x1f;
        int warpid = tid >> 5;
        constexpr int nwarps = BlockSize >> 5;

        // Set up vertex list for shape
        int nCeil = (((n - 1) / BlockSize) + 1) * BlockSize;
        int blockStart = 0;
        for (int i = tid; blockStart < vertexListSize() && i < nCeil; i += BlockSize)
        {
            int vertex = i + start;
            bool collision = i < n && boundingBoxCollision(boundingBoxes[vertex], boundingBox);

            // Count the number of collisions in this warp and store in shared memory
            auto mask = __ballot_sync(0xffffffff, collision);
            if (threadInWarp == 0)
            {
                intShared[warpid] = __popc(mask);
            }
            __syncthreads();

            // Compute partial sum counts for warps
            if (warpid == 0)
            {
                int a = intShared[threadInWarp];
                for (int j = 1; j < nwarps; j <<= 1)
                {
                    int b = __shfl_up_sync(0xffffffff, a, j);
                    if (threadInWarp >= j)
                    {
                        a += b;
                    }
                }
                partialSums[threadInWarp] = blockStart + a;
            }

            __syncthreads();

            // Now we can calculate where to put the index, if it's a potential collision
            if (collision)
            {
                int warpStart = (warpid > 0) ? partialSums[warpid - 1] : blockStart;
                unsigned int lanemask = (1UL << threadInWarp) - 1;
                int index = warpStart + __popc(mask & lanemask);
                if (index < vertexListSize())
                {
                    vertexList[index] = vertex;
                }
            }

            blockStart = partialSums[nwarps - 1];
        }

        // Update number of colliders in A and the total number of pairs
        return min(blockStart, vertexListSize());
    }

    template<int BlockSize>
    __device__ int kernelPopulateVertexList(
        int start,
        int n,
        int tid,
        int* vertexList
    )
    {
        int size = min(n, vertexListSize());
        if (tid < size)
        {
            vertexList[tid] = start + tid;
        }
        return size;
    }

    template<typename T>
    __device__ constexpr int BoneCount();
    template<>
    __device__ constexpr int BoneCount<CudaPerVertexShape>() { return 4; }
    template<>
    __device__ constexpr int BoneCount<CudaPerTriangleShape>() { return 12; }

    __device__ uint32_t getBone(const cuVertex* vertexSetup, const cuPerVertexInput& collider, int i)
    {
        return vertexSetup[collider.vertexIndex].bones[i];
    }

    __device__ uint32_t getBone(const cuVertex* vertexSetup, const cuPerTriangleInput& collider, int i)
    {
        int index = (i < 4) ? collider.vertexIndices.a
            : (i < 8) ? collider.vertexIndices.b
            : collider.vertexIndices.c;

        return vertexSetup[index].bones[i & 3];
    }

    __device__ float getBoneWeight(const cuVertex* vertexSetup, const cuPerVertexInput& collider, int i)
    {
        return vertexSetup[collider.vertexIndex].weights[i];
    }

    __device__ float getBoneWeight(const cuVertex* vertexSetup, const cuPerTriangleInput& collider, int i)
    {
        int index = (i < 4) ? collider.vertexIndices.a
            : (i < 8) ? collider.vertexIndices.b
            : collider.vertexIndices.c;

        return vertexSetup[index].weights[i & 3];
    }

    // kernelCollision does the supporting work for threading the collision checks and making sure that only
    // the deepest result is kept.
    template <cuPenetrationType penType = eNone, typename T, int BlockSize = collisionBlockSize<T>()>
    __global__ void __launch_bounds__(collisionBlockSize<T>(), 1024 / collisionBlockSize<T>()) kernelCollision(
        int n,
        bool swap,
        const cuCollisionSetup* __restrict__ setup,
        const cuColliderData<CudaPerVertexShape> inA,
        const cuColliderData<T> inB,
        const cuCollisionBodyData bodyA,
        const cuCollisionBodyData bodyB,
        cuMergeBuffer mergeBuffer)
    {
        static_assert(vertexListSize() <= BlockSize, "Vertex list must be smaller than block size");

        __shared__ float floatShared[64 + 2 * vertexListSize()];
        int* intShared = reinterpret_cast<int*>(floatShared);

        int tid = threadIdx.x;
        int threadInWarp = tid & 0x1f;
        int warpid = tid >> 5;
        constexpr int nwarps = BlockSize >> 5;
        
        for (int block = blockIdx.x; block < n; block += gridDim.x)
        {
            int nA = setup[block].sizeA;
            int nB = setup[block].sizeB; 
            int offsetA = setup[block].offsetA;
            int offsetB = setup[block].offsetB;

            // Depth should always be negative for collisions. We'll use positive values to signify no
            // collision, and later for mutual exclusion.
            cuCollisionResult temp;
            temp.depth = 1;

            int* vertexListA = intShared + 64;
            int* vertexListB = vertexListA + vertexListSize();

            // Calculate or populate vertex lists, if the number of possible pairs is large. Start with the
            // larger one, and only do the second if the number of pairs is still too high.
            bool order = nA > nB;
            if (order)
            {
                if (nA * nB > BlockSize * vertexListThresholdFactor())
                {
                    nA = kernelComputeVertexList<BlockSize>(
                        offsetA,
                        nA,
                        tid,
                        inA.boundingBoxes,
                        setup[block].boundingBoxB,
                        intShared,
                        vertexListA);
                }
                else
                {
                    nA = kernelPopulateVertexList<BlockSize>(offsetA, nA, tid, vertexListA);
                }
            }

            if (nA * nB > BlockSize * vertexListThresholdFactor())
            {
                nB = kernelComputeVertexList<BlockSize>(
                    offsetB,
                    nB,
                    tid,
                    inB.boundingBoxes,
                    setup[block].boundingBoxA,
                    intShared,
                    vertexListB);
            }
            else
            {
                nB = kernelPopulateVertexList<BlockSize>(offsetB, nB, tid, vertexListB);
            }

            if (!order)
            {
                if (nA * nB > BlockSize * vertexListThresholdFactor())
                {
                    nA = kernelComputeVertexList<BlockSize>(
                        offsetA,
                        nA,
                        tid,
                        inA.boundingBoxes,
                        setup[block].boundingBoxB,
                        intShared,
                        vertexListA);
                }
                else
                {
                    nA = kernelPopulateVertexList<BlockSize>(offsetA, nA, tid, vertexListA);
                }
            }

            // kernelComputeVertexList doesn't do a final synchronize, because it's OK to run it
            // sequentially for both lists without synchronizing between them. So we need to synchronize
            // now to make sure the vertex lists are fully visible.
            __syncthreads();

            int nPairs = nA * nB;

            for (int i = tid; i < nPairs; i += BlockSize)
            {
                int iA = vertexListA[i % nA];
                int iB = vertexListB[i / nA];

                // Skip pairs until we find one with a bounding box collision. This should increase the
                // number of full checks done in parallel, and reduce divergence overall. Note we only do
                // this at all if there are more pairs than threads - if there's only enough work for a
                // single iteration (very common), there's no benefit to trying to reduce it.
#ifdef GPU_BOUNDING_BOX_CHECK
                if (nPairs > BlockSize)
                {
                    while (i < nPairs && !boundingBoxCollision(boundingBoxesA[iA], boundingBoxesB[iB]))
                    {
                        i += BlockSize;
                        if (i < nPairs)
                        {
                            iA = vertexListA[i % nA];
                            iB = vertexListB[i / nA];
                        }
                    }
                }
#endif

                if (i < nPairs && collidePair<penType>(
                    inA.input[iA], inB.input[iB],
                    bodyA.vertexBuffer, bodyB.vertexBuffer,
                    inA.margin, inB.margin,
                    temp))
                {
                    temp.colliderA = iA;
                    temp.colliderB = iB;
                }
            }

            // Find minimum depth in this warp and store in shared memory
            float d = temp.depth;
            for (int j = 16; j > 0; j >>= 1)
            {
                d = min(d, __shfl_down_sync(0xffffffff, d, j));
            }
            if (threadInWarp == 0)
            {
                floatShared[warpid] = d;
            }
            __syncthreads();

            // Find minimum across warps
            if (warpid == 0)
            {
                d = floatShared[threadInWarp];
                for (int j = nwarps >> 1; j > 0; j >>= 1)
                {
                    d = min(d, __shfl_down_sync(0xffffffff, d, j));
                }
                if (threadInWarp == 0)
                {
                    floatShared[0] = d;
                }
            }
            __syncthreads();

            if (floatShared[0] > -FLT_EPSILON)
            {
                return;
            }

            // If the depth of this thread is equal to the minimum, try to set the result. Do an atomic
            // exchange with the first value to ensure that only one thread gets to do this in case of ties.
            cuCollisionResult* result = reinterpret_cast<cuCollisionResult*>(floatShared + 32);
            if (floatShared[0] == temp.depth && atomicExch(floatShared, 2) == temp.depth)
            {
                *result = temp;
            }

            __syncthreads();

            // Update cumulative values in the merge buffer. Use the first two warps, each processing eight
            // or twenty-four entries, depending on the type of collision.
            int indexA = threadIdx.x >> 4;
            int indexB = threadIdx.x & 0x0f;
            if (indexA < BoneCount<CudaPerVertexShape>() && indexB < BoneCount<T>())
            {
                uint32_t boneA = getBone(bodyA.vertexData, inA.input[result->colliderA], indexA);
                uint32_t boneB = getBone(bodyB.vertexData, inB.input[result->colliderB], indexB);

                float weightA = getBoneWeight(bodyA.vertexData, inA.input[result->colliderA], indexA);
                float weightB = getBoneWeight(bodyB.vertexData, inB.input[result->colliderB], indexB);

                if (weightA <= bodyA.boneWeights[boneA] || weightB <= bodyB.boneWeights[boneB])
                {
                    return;
                }

                float flexible = max(inA.input[result->colliderA].flexible, inB.input[result->colliderB].flexible);

                float w = flexible * result->depth;
                float w2 = w * w;

                int i = swap ? boneB : boneA;
                int i_map = swap ? bodyB.boneMap[boneB] : bodyA.boneMap[boneA];
                int j = swap ? boneA : boneB;
                int j_map = swap ? bodyA.boneMap[boneA] : bodyB.boneMap[boneB];

                cuCollisionMerge* c;

                if (i_map == -1 && j_map != -1)
                {
                    c = mergeBuffer.buffer + mergeBuffer.dynx * mergeBuffer.y + mergeBuffer.x * j_map + i;
                }
                else if (i_map != -1)
                {
                    c = mergeBuffer.buffer + i_map * mergeBuffer.y + j;
                }
                else
                {
                    return;
                }

                atomicAdd(&c->weight, w2);

                if (swap)
                {
                    atomicAdd(&c->normal.x, -result->normOnB.x * w * w2);
                    atomicAdd(&c->normal.y, -result->normOnB.y * w * w2);
                    atomicAdd(&c->normal.z, -result->normOnB.z * w * w2);
                    atomicAdd(&c->normal.w, -result->normOnB.w * w * w2);
                    atomicAdd(&c->posA.x, result->posB.x * w2);
                    atomicAdd(&c->posA.y, result->posB.y * w2);
                    atomicAdd(&c->posA.z, result->posB.z * w2);
                    atomicAdd(&c->posA.w, result->posB.w * w2);
                    atomicAdd(&c->posB.x, result->posA.x * w2);
                    atomicAdd(&c->posB.y, result->posA.y * w2);
                    atomicAdd(&c->posB.z, result->posA.z * w2);
                    atomicAdd(&c->posB.w, result->posA.w * w2);
                }
                else
                {
                    atomicAdd(&c->normal.x, result->normOnB.x * w * w2);
                    atomicAdd(&c->normal.y, result->normOnB.y * w * w2);
                    atomicAdd(&c->normal.z, result->normOnB.z * w * w2);
                    atomicAdd(&c->normal.w, result->normOnB.w * w * w2);
                    atomicAdd(&c->posA.x, result->posA.x * w2);
                    atomicAdd(&c->posA.y, result->posA.y * w2);
                    atomicAdd(&c->posA.z, result->posA.z * w2);
                    atomicAdd(&c->posA.w, result->posA.w * w2);
                    atomicAdd(&c->posB.x, result->posB.x * w2);
                    atomicAdd(&c->posB.y, result->posB.y * w2);
                    atomicAdd(&c->posB.z, result->posB.z * w2);
                    atomicAdd(&c->posB.w, result->posB.w * w2);
                }
            }
        }
    }

    __global__ void fullInternalUpdate(
        cuBodyData vertexData,
        const cuBone* __restrict__ boneData,
        cuColliderData<CudaPerVertexShape> perVertexData,
        int nVertexNodes,
        const std::pair<int, int>* __restrict__ vertexNodeData,
        cuAabb* vertexNodeOutput,
        cuColliderData<CudaPerTriangleShape> perTriangleData,
        int nTriangleNodes,
        const std::pair<int, int>* __restrict__ triangleNodeData,
        cuAabb* triangleNodeOutput )
    {
        if (threadIdx.x == 0)
        {
            // Each warp of 32 threads processes 8 vertices sequentially, so we need 4 threads per vertex
            int nBodyBlocks = (vertexData.numVertices * 4 - 1) / cuMapBlockSize() + 1;
            kernelBodyUpdate <<<nBodyBlocks, cuMapBlockSize()>>> (vertexData, boneData);

            constexpr int warpsPerBlock = cuReduceBlockSize() >> 5;

            // Collider calculations also use 4 threads per vertex, but we don't increase the number of
            // blocks so the initial setup doesn't have to be repeated for every collider.
            if (perVertexData.numColliders > 0)
            {
                int nVertexBlocks = (perVertexData.numColliders - 1) / cuMapBlockSize() + 1;
                kernelPerVertexUpdate <<<nVertexBlocks, cuMapBlockSize(), 0>>> (perVertexData, vertexData.vertexBuffer);
                int nReduceBlocks = ((nVertexNodes - 1) / warpsPerBlock) + 1;
                kernelBoundingBoxReduce <<<nReduceBlocks, cuReduceBlockSize(), 0>>> (nVertexNodes, vertexNodeData, perVertexData.boundingBoxes, vertexNodeOutput);
            }
            if (perTriangleData.numColliders > 0)
            {
                int nTriangleBlocks = (perTriangleData.numColliders - 1) / cuMapBlockSize() + 1;
                kernelPerTriangleUpdate <<<nTriangleBlocks, cuMapBlockSize(), 0>>> (perTriangleData, vertexData.vertexBuffer);
                int nReduceBlocks = ((nTriangleNodes - 1) / warpsPerBlock) + 1;
                kernelBoundingBoxReduce <<<nReduceBlocks, cuReduceBlockSize(), 0>>> (nTriangleNodes, triangleNodeData, perTriangleData.boundingBoxes, triangleNodeOutput);
            }
        }
    }

    cuResult cuCreateStream(void** ptr)
    {
        *ptr = new cudaStream_t;
        return cudaStreamCreate(reinterpret_cast<cudaStream_t*>(*ptr));
    }

    void cuDestroyStream(void* ptr)
    {
        cudaStreamDestroy(*reinterpret_cast<cudaStream_t*>(ptr));
        delete reinterpret_cast<cudaStream_t*>(ptr);
    }

    cuResult cuGetDeviceBuffer(void** buf, int size)
    {
        return cudaMalloc(buf, size);
    }

    cuResult cuGetHostBuffer(void** buf, int size)
    {
        return cudaMallocHost(buf, size);
    }

    void cuFreeDevice(void* buf)
    {
        cudaFree(buf);
    }

    void cuFreeHost(void* buf)
    {
        cudaFreeHost(buf);
    }

    cuResult cuCopyToDevice(void* dst, void* src, size_t n, void* stream)
    {
        cudaStream_t* s = reinterpret_cast<cudaStream_t*>(stream);
        return cudaMemcpyAsync(dst, src, n, cudaMemcpyHostToDevice, *s);
    }

    cuResult cuCopyToHost(void* dst, void* src, size_t n, void* stream)
    {
        cudaStream_t* s = reinterpret_cast<cudaStream_t*>(stream);
        return cudaMemcpyAsync(dst, src, n, cudaMemcpyDeviceToHost, *s);
    }

    cuResult cuMemset(void* buf, int value, size_t n, void* stream)
    {
        cudaStream_t* s = reinterpret_cast<cudaStream_t*>(stream);
        return cudaMemsetAsync(buf, value, n, *s);
    }

    template<cuPenetrationType penType, typename T>
    cuResult cuRunCollision(
        void* stream,
        int n,
        bool swap,
        cuCollisionSetup* setup,
        cuColliderData<CudaPerVertexShape> inA,
        cuColliderData<T> inB,
        cuCollisionBodyData bodyA,
        cuCollisionBodyData bodyB,
        cuMergeBuffer mergeBuffer)
    {
        cudaStream_t* s = reinterpret_cast<cudaStream_t*>(stream);

        kernelCollision<penType> <<<n, collisionBlockSize<T>(), 0, *s >>> (
            n, swap, setup, inA, inB, bodyA, bodyB, mergeBuffer);
        return cuResult();
    }

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
        cuAabb* triangleNodeOutput)
    {
        cudaStream_t* s = reinterpret_cast<cudaStream_t*>(stream);

        fullInternalUpdate <<<1, 1, 0, *s >>> (
            vertexData,
            boneData,
            perVertexData,
            nVertexNodes,
            vertexNodeData,
            vertexNodeOutput,
            perTriangleData,
            nTriangleNodes,
            triangleNodeData,
            triangleNodeOutput);
        return cuResult();
    }

    cuResult cuSynchronize(void* stream)
    {
        cudaStream_t* s = reinterpret_cast<cudaStream_t*>(stream);

        if (s)
        {
            return cudaStreamSynchronize(*s);
        }
        else
        {
            return cudaDeviceSynchronize();
        }
    }

    cuResult cuCreateEvent(void** ptr)
    {
        *ptr = new cudaEvent_t;
        return cudaEventCreate(reinterpret_cast<cudaEvent_t*>(*ptr));
    }

    void cuDestroyEvent(void* ptr)
    {
        cudaEventDestroy(*reinterpret_cast<cudaEvent_t*>(ptr));
        delete reinterpret_cast<cudaEvent_t*>(ptr);
    }

    void cuRecordEvent(void* ptr, void* stream)
    {
        cudaEvent_t* e = reinterpret_cast<cudaEvent_t*>(ptr);
        cudaStream_t* s = reinterpret_cast<cudaStream_t*>(stream);
        cudaEventRecord(*e, *s);
    }

    void cuWaitEvent(void* ptr)
    {
        cudaEvent_t* e = reinterpret_cast<cudaEvent_t*>(ptr);
        cudaEventSynchronize(*e);
    }

    void cuInitialize()
    {
//        cudaSetDeviceFlags(cudaDeviceScheduleYield);
    }

    int cuDeviceCount()
    {
        int count;
        cudaGetDeviceCount(&count);
        return count;
    }

    void cuSetDevice(int id)
    {
        cudaSetDevice(id);
    }

    int cuGetDevice()
    {
        int id;
        cudaGetDevice(&id);
        return id;
    }

    void* cuDevicePointer(void* ptr)
    {
        void* p;
        cudaHostGetDevicePointer(&p, ptr, 0);
        return p;
    }

    template cuResult cuRunCollision<eNone, CudaPerVertexShape>(
        void*, int, bool, cuCollisionSetup*, cuColliderData<CudaPerVertexShape>, cuColliderData<CudaPerVertexShape>,
        cuCollisionBodyData, cuCollisionBodyData, cuMergeBuffer);
    template cuResult cuRunCollision<eNone, CudaPerTriangleShape>(
        void*, int, bool, cuCollisionSetup*, cuColliderData<CudaPerVertexShape>, cuColliderData<CudaPerTriangleShape>,
        cuCollisionBodyData, cuCollisionBodyData, cuMergeBuffer);
    template cuResult cuRunCollision<eInternal, CudaPerTriangleShape>(
        void*, int, bool, cuCollisionSetup*, cuColliderData<CudaPerVertexShape>, cuColliderData<CudaPerTriangleShape>,
        cuCollisionBodyData, cuCollisionBodyData, cuMergeBuffer);
}