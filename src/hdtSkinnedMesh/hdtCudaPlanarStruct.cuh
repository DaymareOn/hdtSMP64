#pragma once

#ifndef __NVCC__
#define __host__
#define __device__
#define __forceinline__ __forceinline
#define __restrict__
#endif

namespace hdt
{
    template<typename StructT, typename... Args>
    struct InterleavedStruct;

    template<typename StructT, typename... Args>
    struct PlanarStruct;

    template<typename... Args>
    using ArrayType = InterleavedStruct<Args...>;

    template<typename StructT, typename...>
    struct InterleavedStruct
    {
        using type = StructT;

        __host__ __device__ __forceinline__ InterleavedStruct(StructT* buffer, int)
            : m_buffer(buffer)
        {}

        __host__ __device__ __forceinline__ StructT& operator[](int i)
        {
            return m_buffer[i];
        }

        __host__ __device__ __forceinline__ const StructT& operator[](int i) const
        {
            return m_buffer[i];
        }

    private:

        StructT* __restrict__ const m_buffer;
    };

    template<int... Vals>
    struct PackSum;
    template<>
    struct PackSum<>
    {
        static constexpr int val = 0;
    };
    template<int First, int... Rest>
    struct PackSum<First, Rest...>
    {
        static constexpr int val = First + PackSum<Rest...>::val;
    };

    template<typename T>
    struct SizeHelper
    {
        static constexpr int count = 1;
        static constexpr int size = sizeof(T);
    };
    template<typename StructT, typename... Args>
    struct SizeHelper<PlanarStruct<StructT, Args...>>
    {
        static constexpr int count = PackSum<SizeHelper<Args>::count...>::val;
        static constexpr int size = PackSum<SizeHelper<Args>::size...>::val;
    };

    template<int N, typename... Args>
    struct OffsetHelper;
    template<typename... Args>
    struct OffsetHelper<0, Args...>
    {
        static constexpr int index = 0;
        static constexpr int offset = 0;
    };
    template<int N, typename First, typename... Args>
    struct OffsetHelper<N, First, Args...>
    {
        static constexpr int index = SizeHelper<First>::count + OffsetHelper<N - 1, Args...>::index;
        static constexpr int offset = SizeHelper<First>::size + OffsetHelper<N - 1, Args...>::offset;
    };

    template<int N, typename... Args>
    struct TypeHelper;
    template<typename T, typename... Args>
    struct TypeHelper<0, T, Args...>
    {
        using type = T;
    };
    template<int N, typename T, typename... Args>
    struct TypeHelper<N, T, Args...>
    {
        using type = typename TypeHelper<N - 1, Args...>::type;
    };

    template<typename... Args>
    struct Flattener;
    template<typename... Args>
    struct Merge;
    template<typename T>
    struct Merge<T>
    {
        using type = T;
    };
    template<typename... Ts1, typename... Ts2, typename... Args>
    struct Merge<Flattener<Ts1...>, Flattener<Ts2...>, Args...>
    {
        using type = typename Merge<Flattener<Ts1..., Ts2...>, Args...>::type;
    };
    template<typename T>
    struct Flattener<T>
    {
        using type = Flattener<T>;
    };
    template<typename StructT, typename... Args>
    struct Flattener<PlanarStruct<StructT, Args...>>
    {
        using type = typename Merge<typename Flattener<Args>::type...>::type;
    };

    template<typename... Args>
    struct PlanarStructPointers;

    template<>
    struct PlanarStructPointers<>
    {
        PlanarStructPointers(uint8_t*, int)
        {}
    };

    template<typename First, typename... Rest>
    struct PlanarStructPointers<First, Rest...>
    {
        PlanarStructPointers(uint8_t* buffer, int size)
            : m_ptr(reinterpret_cast<First*>(buffer)),
            m_rest(buffer + size * sizeof(First), size)
        {}

        First* const m_ptr;
        PlanarStructPointers<Rest...> m_rest;
    };

    template<typename T, int I>
    struct PointerGetter;

    template<typename First, typename... Rest>
    struct PointerGetter<PlanarStructPointers<First, Rest...>, 0>
    {
        __host__ __device__ __forceinline__ static First* get(const PlanarStructPointers<First, Rest...>& ptrs) { return ptrs.m_ptr; }
    };

    template<typename First, typename... Rest, int I>
    struct PointerGetter<PlanarStructPointers<First, Rest...>, I>
    {
        __host__ __device__ __forceinline__ static auto get(const PlanarStructPointers<First, Rest...>& ptrs)
        {
            return PointerGetter<PlanarStructPointers<Rest...>, I - 1>::get(ptrs.m_rest);
        }
    };

    template<typename StructT, typename FlatT>
    struct ToPlanar;
    template<typename StructT, typename... Args>
    struct ToPlanar<StructT, Flattener<Args...>>
    {
        using type = PlanarStruct<StructT, Args...>;
        using ptrType = PlanarStructPointers<Args...>;
    };

    template<typename BaseT, typename StructT, int Offset>
    struct PlanarAccessor;

    template<typename StructT, typename... Args>
    struct PlanarStruct
    {
        __host__ __device__ __forceinline__ PlanarStruct(StructT* buffer, int size)
            : m_pointers(reinterpret_cast<uint8_t*>(buffer), size)
        {}

        using type = StructT;
        using flatType = typename ToPlanar<StructT, typename Flattener<PlanarStruct>::type>::type;
        using ptrType = typename ToPlanar<StructT, typename Flattener<PlanarStruct>::type>::ptrType;
        using AccessorT = PlanarAccessor<PlanarStruct, PlanarStruct, 0>;
        static constexpr int count = sizeof...(Args);

        template<int N>
        struct ElementData
        {
            using type = typename TypeHelper<N, Args...>::type;
            static constexpr int index = OffsetHelper<N, Args...>::index;
            static constexpr int offset = OffsetHelper<N, Args...>::offset;
        };

        __host__ __device__ __forceinline__ AccessorT operator[](int i)
        {
            return AccessorT(*this, i);
        }

        __host__ __device__ __forceinline__ const AccessorT operator[](int i) const
        {
            return AccessorT(*this, i);
        }

        ptrType m_pointers;
    };

    template<typename BaseT, typename StructT, typename T, int Offset>
    struct Getter
    {
        __host__ __device__ __forceinline__ static T& get(const BaseT& s, const int i)
        {
            return PointerGetter<typename BaseT::ptrType, Offset>::get(s.m_pointers)[i];
        }
    };

    template<typename BaseT, typename StructT, typename T, typename... Ts, int Offset>
    struct Getter<BaseT, StructT, PlanarStruct<T, Ts...>, Offset>
    {
        __host__ __device__ __forceinline__ static PlanarAccessor<BaseT, PlanarStruct<T, Ts...>, Offset> get(const BaseT& s, const int i)
        {
            return PlanarAccessor<BaseT, PlanarStruct<T, Ts...>, Offset>(s, i);
        }
    };

    template<typename BaseT, typename StructT, int Offset>
    struct PlanarAccessor
    {
        __host__ __device__ __forceinline__ PlanarAccessor(const BaseT& s, const int i)
            : m_s(s), m_i(i)
        {}

        __host__ __device__ __forceinline__ operator typename StructT::type()
        {
            return toStruct(std::make_index_sequence<StructT::count>());
        }

        __host__ __device__ __forceinline__ operator const typename StructT::type() const
        {
            return toStruct(std::make_index_sequence<StructT::count>());
        }

        __host__ __device__ __forceinline__ PlanarAccessor& operator=(const PlanarAccessor& o)
        {
            assign(o, std::make_index_sequence<StructT::count>());
            return *this;
        }

        template<typename T, int O>
        __host__ __device__ __forceinline__ PlanarAccessor& operator=(const PlanarAccessor<T, StructT, O>& o)
        {
            assign(o, std::make_index_sequence<StructT::count>());
            return *this;
        }

        __host__ __device__ __forceinline__ PlanarAccessor& operator=(const typename StructT::type& o)
        {
            assign(o, std::make_index_sequence<StructT::flatType::count>());
            return *this;
        }

    private:

        template<int N>
        __host__ __device__ __forceinline__ auto get() -> decltype(Getter<BaseT, StructT, typename StructT::template ElementData<N>::type, Offset + StructT::template ElementData<N>::index>::get(std::declval<BaseT>(), std::declval<int>()))
        {
            return Getter<BaseT, StructT, typename StructT::template ElementData<N>::type, Offset + StructT::template ElementData<N>::index>::get(m_s, m_i);
        }

        template<int N>
        __host__ __device__ __forceinline__ auto get() const -> const decltype(Getter<BaseT, StructT, typename StructT::template ElementData<N>::type, Offset + StructT::template ElementData<N>::index>::get(std::declval<BaseT>(), std::declval<int>()))
        {
            return Getter<BaseT, StructT, typename StructT::template ElementData<N>::type, Offset + StructT::template ElementData<N>::index>::get(m_s, m_i);
        }

        template<typename... Ts>
        __host__ __device__ __forceinline__ static void dummy(Ts&&...) {}

        template<int... Ints>
        __host__ __device__ __forceinline__ typename StructT::type toStruct(std::index_sequence<Ints...>)
        {
            return { get<Ints>()... };
        }

        template<int... Ints>
        __host__ __device__ __forceinline__ typename const StructT::type toStruct(std::index_sequence<Ints...>) const
        {
            return { get<Ints>()... };
        }

        template<typename T, int O, int... Ints>
        __host__ __device__ __forceinline__ void assign(const PlanarAccessor<T, StructT, O>& o, std::index_sequence<Ints...>)
        {
            dummy(get<Ints>() = o.get<Ints>()...);
        }

        template<int... Ints>
        __host__ __device__ __forceinline__ void assign(const typename StructT::type& o, std::index_sequence<Ints...>)
        {
            dummy(Getter<BaseT, typename StructT::flatType, typename StructT::flatType::template ElementData<Ints>::type, Offset + StructT::flatType::template ElementData<Ints>::index>::get(m_s, m_i)
                = *reinterpret_cast<const typename StructT::flatType::template ElementData<Ints>::type*>(reinterpret_cast<const uint8_t*>(&o) + StructT::flatType::template ElementData<Ints>::offset)...);
        }

        const BaseT& m_s;
        const int m_i;
    };
}
