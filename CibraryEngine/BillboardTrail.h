#pragma once
#include "StdAfx.h"

#include "Vector.h"
#include "Texture2D.h"
#include "GameState.h"
#include "Entity.h"

#include "SceneRenderer.h"

namespace CibraryEngine
{
	using namespace std;

	class BillboardMaterial;

	class BillboardTrail : public Entity
	{
		public:

			struct TrailNode
			{
				Vec3 pos;

				float age, max_age;

				TrailNode() { }
				TrailNode(Vec3 pos, float age, float max_age) : pos(pos), age(age), max_age(max_age) { }
			};

			struct TrailHead { virtual bool operator()(TrailNode& node) = 0; };

		private:

			unsigned int node_count;
			vector<TrailNode> trail;
			vector<int> nu_trail;				// used to save on new/delete

			BillboardMaterial* material;
			float width;

			float precision, precision_sq;
			bool shrinking;

			Sphere bs;

			bool AddNode();
			void InvalidateBoundingSphere();

		public:

			BillboardTrail(GameState* gs, TrailHead* trailhead, BillboardMaterial* material, float width, float precision);
			void Update(TimingInfo time);
			void Vis(SceneRenderer* renderer);

			TrailHead* trailhead;

			Sphere GetBoundingSphere();		
	};
}
