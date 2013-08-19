#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	using namespace std;

	// "smart" in that it doesn't deallocate the memory when it erases or clears stuff
	// but also "dumb" in that it never rehashes
	template<class T, unsigned int N> struct SmartHashSet
	{
		vector<T*> buckets[N];
		unsigned int count;

		static const unsigned int hash_size = N;

		SmartHashSet() : buckets(), count(0) { }

		void Insert(T* item)
		{
			unsigned int hash = ((unsigned int)item / sizeof(T)) % N;

			vector<T*>& bucket = buckets[hash];
			for(T **iter = bucket.data(), **bucket_end = iter + bucket.size(); iter != bucket_end; ++iter)
				if(*iter == item)
					return;

			bucket.push_back(item);
			++count;
		}

		void Erase(T* item)
		{
			unsigned int hash = ((unsigned int)item / sizeof(T)) % N;

			vector<T*>& bucket = buckets[hash];
			for(T **iter = bucket.data(), **bucket_end = iter + bucket.size(); iter != bucket_end; ++iter)
				if(*iter == item)
				{
					// replace this element with the last one in the array
					--bucket_end;
					*iter = *bucket_end;			
					bucket.pop_back();

					assert(count);
					--count;

					return;
				}
		}

		bool Contains(T* item)
		{
			unsigned int hash = ((unsigned int)item / sizeof(T)) % N;

			vector<T*>& bucket = buckets[hash];
			for(T **iter = bucket.data(), **bucket_end = iter + bucket.size(); iter != bucket_end; ++iter)
				if(*iter == item)
					return true;

			return false;
		}

		void Clear()
		{
			for(vector<T*>* iter = buckets, *buckets_end = buckets + N; iter != buckets_end; ++iter)
				iter->clear();

			count = 0;
		}
	};
}
