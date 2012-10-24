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
			for(vector<T*>::iterator iter = bucket.begin(), bucket_end = bucket.end(); iter != bucket_end; ++iter)
				if(*iter == item)
					return;

			bucket.push_back(item);
			++count;
		}

		void Erase(T* item)
		{
			if(count)
			{
				unsigned int hash = ((unsigned int)item / sizeof(T)) % N;

				vector<T*>& bucket = buckets[hash];
				for(unsigned int i = 0, bucket_size = bucket.size(); i < bucket_size; ++i)
					if(bucket[i] == item)
					{
						bucket[i] = bucket[bucket_size - 1];			// replace this element with the last one in the array
						bucket.pop_back();

						assert(count);
						--count;

						return;
					}
			}
		}

		bool Contains(T* item)
		{
			if(count)
			{
				unsigned int hash = ((unsigned int)item / sizeof(T)) % N;

				vector<T*>& bucket = buckets[hash];
				for(unsigned int i = 0, bucket_size = bucket.size(); i < bucket_size; ++i)
					if(bucket[i] == item)
						return true;
			}

			return false;
		}

		void Clear()
		{
			if(count)
			{
				for(unsigned int i = 0; i < hash_size; ++i)
					buckets[i].clear();

				count = 0;
			}
		}
	};
}
