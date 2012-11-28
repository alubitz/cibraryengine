#pragma once

#include "StdAfx.h"
#include <typeinfo>

#include "DebugLog.h"
#include "ContentMetadata.h"

namespace CibraryEngine
{
	using namespace std;

	struct ContentMan;
	class ContentTypeHandlerBase;
	template <class T> class ContentTypeHandler;

	template<class T> struct ContentHandle;

	struct MetaDataPair
	{
		ContentMetadata meta;
		void* data;

		MetaDataPair() : meta(), data(NULL) { }
		MetaDataPair(ContentMetadata meta, void* data) : meta(meta), data(data) { }
	};

	struct CacheBase
	{
		map<unsigned int, MetaDataPair> content;
		ContentMan* man;
		unsigned int next_int;

		ContentTypeHandlerBase* handler;

		CacheBase(ContentMan* man) : content(), man(man), next_int(1) { }
	};

	template <class T> struct Cache : public CacheBase
	{
		Cache(ContentMan* man);
		Cache(ContentMan* man, ContentTypeHandler<T>* handler);

		T*& GetContent(unsigned int id);
		ContentMetadata& GetMetadata(unsigned int id);

		ContentHandle<T> CreateHandle(ContentMetadata meta);
		ContentHandle<T> GetHandle(const string& name);

		void ForceLoad(ContentHandle<T> handle);
		void Unload(ContentHandle<T> handle);
		bool IsLoaded(ContentHandle<T> handle);

		T* Load(const string& asset_name);

		ContentTypeHandler<T>* GetHandler();
		void SetHandler(ContentTypeHandler<T>* handler);
	};
}

#include "ContentTypeHandler.h"

namespace CibraryEngine
{
	/*
	 * Cache<T> method implementations
	 */
	template <class T> Cache<T>::Cache(ContentMan* man) : CacheBase(man) { }
	template <class T> Cache<T>::Cache(ContentMan* man, ContentTypeHandler<T>* handler_) : CacheBase(man) { handler = handler_; }

	template <class T> ContentMetadata& Cache<T>::GetMetadata(unsigned int id)		{ return content[id].meta; }

	template <class T> T*& Cache<T>::GetContent(unsigned int id)					{ void*& vpref = content[id].data; return (T*&)vpref; }

	template <class T> ContentHandle<T> Cache<T>::CreateHandle(ContentMetadata meta)
	{
		ContentHandle<T> handle(this, next_int++);
		content[handle.id] = MetaDataPair(meta, NULL);
		return handle;
	}

	template <class T> ContentHandle<T> Cache<T>::GetHandle(const string& name)
	{
		for(map<unsigned int, MetaDataPair>::iterator iter = content.begin(); iter != content.end(); ++iter)
			if(iter->second.meta.name == name)
				return ContentHandle<T>(this, iter->first);

		return CreateHandle(ContentMetadata(name));
	}

	template <class T> void Cache<T>::ForceLoad(ContentHandle<T> handle)
	{
		unsigned int id = handle.id;

		if(content[id].data == NULL)
		{
			ContentMetadata& meta = GetMetadata(id);

			// have we tried and failed to load the associated content?
			if(meta.fail)
				return;

			content[id].data = ((ContentTypeHandler<T>*)handler)->Load(meta);

			if(content[id].data == NULL)
				meta.fail = true;
		}
	}

	template <class T> void Cache<T>::Unload(ContentHandle<T> handle)
	{
		unsigned int id = handle.id;

		if(void* data = content[id].data)
		{
			content[id].data = NULL;

			ContentMetadata& meta = GetMetadata(id);
			((ContentTypeHandler<T>*)handler)->Unload(data, meta);
		}
	}

	template <class T> bool Cache<T>::IsLoaded(ContentHandle<T> handle)				{ return content[handle.id].data != NULL; }

	template <class T> T* Cache<T>::Load(const string& asset_name)
	{
		ContentHandle<T> handle = GetHandle(asset_name);
		ForceLoad(handle);
		return GetContent(handle.id);
	}

	template <class T> ContentTypeHandler<T>* Cache<T>::GetHandler()				{ return handler; }
	template <class T> void Cache<T>::SetHandler(ContentTypeHandler<T>* handler_)	{ handler = handler_; }
}
