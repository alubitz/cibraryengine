#pragma once

#include "StdAfx.h"
#include <typeinfo>

namespace CibraryEngine
{
	using namespace std;

	struct ContentMan;
	struct ContentMetadata;

	struct CacheBase;

	struct ContentHandleBase
	{
		CacheBase* cache;
		unsigned int id;

		ContentHandleBase(CacheBase* cache, unsigned int id) : cache(cache), id(id) { }
	};

	template <class T> struct ContentHandle : public ContentHandleBase
	{
		ContentHandle(Cache<T>* cache, unsigned int id) : ContentHandleBase(cache, id) { }

		T* GetObject();
		ContentMetadata& GetMetadata();
		ContentTypeHandler<T>* GetTypeHandler();
	};
}

#include "ContentMetadata.h"
#include "ContentMan.h"

namespace CibraryEngine
{
	template<class T> T* ContentHandle<T>::GetObject() { return ((Cache<T>*)cache)->GetContent(id); }

	template<class T> ContentMetadata& ContentHandle<T>::GetMetadata() { return ((Cache<T>*)cache)->GetMetadata(id); }

	template<class T> ContentTypeHandler<T>* ContentHandle<T>::GetTypeHandler() { return ((Cache<T>*)cache)->GetHandler(); }
}