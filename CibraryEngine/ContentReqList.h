#pragma once

#include "StdAfx.h"
#include "Disposable.h"

namespace CibraryEngine
{
	using namespace std;

	class ContentMan;
	template<class T> class ContentHandle;

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

			ContentHandle<UberModel> LoadModel(string model_name);

			void LoadContent(string* status);
	};
}
