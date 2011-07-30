#pragma once

#include "StdAfx.h"
#include <typeinfo>

namespace CibraryEngine
{
	using namespace std;

	class ContentMan;
	struct ContentMetadata;

	struct ContentHandleBase
	{
		ContentMan* man;
		unsigned int id;

		ContentHandleBase(ContentMan* man, unsigned int id) : man(man), id(id) { }
	};

	template <class T> struct ContentHandle : public ContentHandleBase
	{
		ContentHandle(ContentMan* man, unsigned int id) : ContentHandleBase(man, id) { }

		T* GetObject();
		ContentMetadata& GetMetadata();
		ContentTypeHandler<T>* GetTypeHandler();
	};
}

#include "ContentMetadata.h"
#include "ContentMan.h"

namespace CibraryEngine
{
	template<class T> T* ContentHandle<T>::GetObject() { return man->GetContent<T>(*this); }

	template<class T> ContentMetadata& ContentHandle<T>::GetMetadata() { return man->GetMetadata<T>(*this); }

	template<class T> ContentTypeHandler<T>* ContentHandle<T>::GetTypeHandler() { return man->GetHandler<T>(); }
}