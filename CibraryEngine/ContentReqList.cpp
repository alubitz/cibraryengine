#include "StdAfx.h"
#include "ContentReqList.h"

#include "UberModel.h"
#include "ModelPhysics.h"

namespace CibraryEngine
{
	using namespace std;

	/*
	 * ContentReqList private implementation struct
	 */
	struct ContentReqList::Imp
	{
		ContentMan* content;

		Cache<UberModel>* ubermodel_cache;
		Cache<ModelPhysics>* mphys_cache;
		list<ContentHandle<UberModel> > models;

		Imp(ContentMan* content) : content(content), ubermodel_cache(content->GetCache<UberModel>()), mphys_cache(content->GetCache<ModelPhysics>()), models() { }
	};




	/*
	 * ContentReqList methods
	 */
	ContentReqList::ContentReqList(ContentMan* content) : imp(new Imp(content)) { }

	void ContentReqList::InnerDispose() { delete imp; }

	ContentHandle<UberModel> ContentReqList::LoadModel(const string& model_name) 
	{
		ContentHandle<UberModel> handle(imp->ubermodel_cache->GetHandle(model_name));
		imp->models.push_back(handle);
		return handle;
	}

	void ContentReqList::LoadContent(string* status)
	{
		for(list<ContentHandle<UberModel> >::iterator iter = imp->models.begin(); iter != imp->models.end(); ++iter)
		{
			ContentHandle<UberModel> handle = *iter;
			*status = "Models... " + handle.GetMetadata().name + ".zzz";

			imp->ubermodel_cache->ForceLoad(handle);
			imp->mphys_cache->Load(handle.GetMetadata().name);
		}
	}
}
