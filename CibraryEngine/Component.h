#pragma once
#include "StdAfx.h"

#include "TimingInfo.h"

namespace CibraryEngine
{
	class SceneRenderer;
	class Entity;

	class Component
	{
		public:

			Entity* entity;

			Component(Entity* entity) : entity(entity) { }
			virtual ~Component() { }

			virtual void Update(const TimingInfo& time) { }
			virtual void Vis(SceneRenderer* renderer) { }
	};
}
