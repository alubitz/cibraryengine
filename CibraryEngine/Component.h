#pragma once
#include "StdAfx.h"

namespace CibraryEngine
{
	struct TimingInfo;

	class SceneRenderer;

	class Component
	{
		public:

			virtual void Update(TimingInfo time);
			virtual void Vis(SceneRenderer* renderer);
	};
}
