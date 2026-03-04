#pragma once

#include "IString.h"

namespace hdt
{
	class StringImpl : public IString
	{
	public:
		StringImpl(size_t hash, std::string&& str);
		virtual ~StringImpl();
		
		virtual const char* cstr() const { return m_str.c_str(); }
		virtual size_t size() const { return m_str.size(); }

		inline size_t hash() const { return m_hash; }
		inline const std::string str() const { return m_str; }

		inline const uint32_t GetRefCount() const noexcept { return _refCount; }
	protected:
		size_t			m_hash;
		std::string		m_str;
	};

	class StringManager final
	{
	public:
		static constexpr size_t BucketCount = 65536;

		//
		struct Bucket
		{
		public:
			StringImpl* get(size_t hash, std::string&& str);
			void clean();
		protected:
			std::vector<RE::BSTSmartPointer<StringImpl>> m_list;
			std::mutex									 m_lock;
		};

	public:
		static StringManager* instance();

		//
		StringImpl* get(const char* begin, const char* end);
	private:
		Bucket							m_buckets[BucketCount];
		RE::BSTSmartPointer<StringImpl> m_empty;
		std::thread						m_gcThread;
		std::atomic_bool				m_gcExit;
	private:
		StringManager();
		~StringManager();
	};
}
