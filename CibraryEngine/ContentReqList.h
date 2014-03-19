#pragma once

#include "StdAfx.h"
#include "Disposable.h"

namespace CibraryEngine
{
	using namespace std;

	struct ContentMan;
	template<class T> struct ContentHandle;

	class UberModel;

	class ContentReqList : public Disposable
	{
		private:

			struct Imp;
			Imp* imp;

		protected:

			void InnerDispose();

		public:

			ContentReqList(ContentMan* content);

			ContentHandle<UberModel> LoadModel(const string& model_name);

			class StatusUpdater { public: virtual void SetString(const string& str) = 0; };

			void LoadContent(StatusUpdater* status, bool materials = true);
	};
}
