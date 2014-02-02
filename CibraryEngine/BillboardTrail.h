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
				TrailNode(const Vec3& pos, float age, float max_age) : pos(pos), age(age), max_age(max_age) { }
			};

			struct TrailHead
			{
				bool trail_free;
				bool head_free;

				TrailHead() : trail_free(false), head_free(false) { }
				virtual ~TrailHead() { }

				virtual bool operator()(TrailNode& node) = 0;
			};

		private:

			unsigned int node_count;
			vector<TrailNode> trail;
			vector<int> nu_trail;				// used to save on new/delete
			bool shrinking;

			BillboardMaterial* material;
			float width;

			Sphere bs;


			bool AddNode();
			void InvalidateBoundingSphere();

		protected:

			void InnerDispose();

		public:

			TrailHead* trailhead;


			BillboardTrail(GameState* gs, TrailHead* trailhead, BillboardMaterial* material, float width);

			void Update(const TimingInfo& time);
			Sphere GetBoundingSphere();	
			
			void Vis(SceneRenderer* renderer);	
	};
}
