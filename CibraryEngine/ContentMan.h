#pragma once

#include "StdAfx.h"
#include <typeinfo>

namespace CibraryEngine
{
	using namespace std;

	template <class T> class ContentTypeHandler;
	struct ContentMetadata;
	struct CacheBase;
	template <class T> struct ContentHandle;

	class ContentMan
	{
		public:

			map<const type_info*, CacheBase*> caches;

			ContentMan();

			template <class T> T* GetContent(ContentHandle<T> handle);
			template <class T> ContentMetadata& GetMetadata(ContentHandle<T> handle);
			template <class T> ContentHandle<T> GetHandle(string name);

			template <class T> ContentTypeHandler<T>* GetHandler();
			template <class T> void SetHandler(ContentTypeHandler<T>* handler);

			template <class T> void ForceLoad(ContentHandle<T> handle);
			template <class T> void Unload(ContentHandle<T> handle);
			template <class T> bool IsLoaded(ContentHandle<T> handle);

			template <class T> T* Load(string asset_name);
	};
}

#include "ContentHandle.h"
#include "ContentMetadata.h"
#include "Cache.h"

namespace CibraryEngine
{

	using namespace std;

	/*
	 * ContentMan template function implementations (non-template stuff is in ContentMan.cpp)
	 */
	template <class T> T* ContentMan::GetContent(ContentHandle<T> handle)
	{
		map<const type_info*, CacheBase*>::iterator found = caches.find(&typeid(T));
		if(found == caches.end())
			return NULL;

		return (T*)((Cache<T>*)found->second)->GetContent(handle.id);
	}

	template <class T> ContentMetadata& ContentMan::GetMetadata(ContentHandle<T> handle)
	{
		map<const type_info*, CacheBase*>::iterator found = caches.find(&typeid(T));
		// if there's no such loader... shit.

		return ((Cache<T>*)found->second)->GetMetadata(handle.id);
	}

	template <class T> ContentHandle<T> ContentMan::GetHandle(string name)
	{
		map<const type_info*, CacheBase*>::iterator found = caches.find(&typeid(T));
		if(found == caches.end())
			return ContentHandle<T>(this, 0);

		Cache<T>* cache = (Cache<T>*)found->second;

		for(std::map<unsigned int, MetaDataPair>::iterator iter = cache->content.begin(); iter != cache->content.end(); iter++)
			if(iter->second.meta.name == name)
				return ContentHandle<T>(this, iter->first);

		ContentMetadata meta;
		meta.name = name;

		return cache->CreateHandle(meta);
	}

	template <class T> ContentTypeHandler<T>* ContentMan::GetHandler()
	{
		const type_info* t = &typeid(T);
		map<const type_info*, CacheBase*>::iterator found = caches.find(t);
		if(found == caches.end())
			return NULL;
		else
			return (ContentTypeHandler<T>*)found->second;
	}

	template <class T> void ContentMan::SetHandler(ContentTypeHandler<T>* handler)
	{
		const type_info* t = &typeid(T);
		map<const type_info*, CacheBase*>::iterator found = caches.find(t);
		if(found == caches.end())
		{
			caches[t] = new CacheBase(this);
			caches[t]->handler = handler;
		}
		else
			found->second->handler = handler;
	}

	template <class T> void ContentMan::ForceLoad(ContentHandle<T> handle)
	{ 
		map<const type_info*, CacheBase*>::iterator found = caches.find(&typeid(T));
		if(found == caches.end())
			return;

		((Cache<T>*)found->second)->ForceLoad(handle);
	}

	template <class T> void ContentMan::Unload(ContentHandle<T> handle)
	{
		map<const type_info*, CacheBase*>::iterator found = caches.find(&typeid(T));
		if(found == caches.end())
			return;

		((Cache<T>*)found->second)->Unload(handle);
	}

	template <class T> bool ContentMan::IsLoaded(ContentHandle<T> handle)
	{
		map<const type_info*, CacheBase*>::iterator found = caches.find(&typeid(T));
		if(found == caches.end())
			return false;				// idk what to do here?

		return ((Cache<T>*)found->second)->IsLoaded(handle);
	}

	template <class T> T* ContentMan::Load(string asset_name)
	{
		ContentHandle<T> handle = GetHandle<T>(asset_name);
		ForceLoad<T>(handle);
		return GetContent<T>(handle);
	}
}