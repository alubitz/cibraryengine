#pragma once

#include "StdAfx.h"
#include <typeinfo>

namespace CibraryEngine
{
	using namespace std;

	template <class T> class ContentTypeHandler;
	struct ContentMetadata;
	struct CacheBase;
	template <class T> struct Cache;
	template <class T> struct ContentHandle;

	struct ContentMan
	{
		map<const type_info*, CacheBase*> caches;

		ContentMan();

		template <class T> Cache<T>* GetCache();
		template <class T> void SetCache(Cache<T>* cache);
		template <class T> Cache<T>* CreateCache(ContentTypeHandler<T>* handler);
	};
}

#include "ContentHandle.h"
#include "ContentMetadata.h"
#include "Cache.h"

namespace CibraryEngine
{

	using namespace std;

	template <class T> Cache<T>* ContentMan::GetCache()
	{
		map<const type_info*, CacheBase*>::iterator found = caches.find(&typeid(T));
		if(found == caches.end())
			return NULL;
		else
			return (Cache<T>*)found->second;
	}

	template <class T> void ContentMan::SetCache(Cache<T>* cache)
	{
		caches[&typeid(T)] = cache;
	}

	template <class T> Cache<T>* ContentMan::CreateCache(ContentTypeHandler<T>* handler)
	{
		Cache<T>* c = new Cache<T>(this, handler);
		caches[&typeid(T)] = c;

		return c;
	}
}