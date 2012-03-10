#pragma once

#include "StdAfx.h"
#include "CollisionShape.h"

#include "Octree.h"

namespace CibraryEngine
{
	struct Ray;
	struct Intersection;

	class TriangleMeshShape : public CollisionShape
	{
		private:

			bool built;

			struct TriCache
			{
				Vec3 a, b, c;
				Vec3 ab, ac;
				Vec3 p, q;
				Plane plane;
				float u_offset, v_offset;
			};
			TriCache* cache;

			struct NodeData
			{
				struct Tri
				{
					unsigned int face_index;
					AABB aabb;

					Tri(unsigned int face_index, AABB aabb) : face_index(face_index), aabb(aabb) { }
				};

				vector<Tri> triangles;
				unsigned int tri_count;

				NodeData() : triangles(), tri_count() { }

				void Add(Tri tri) { triangles.push_back(tri); ++tri_count; }
			};

			Octree<NodeData>* octree;
			void BuildOctree();

			void InitCache();
			void DeleteCache();

			void BuildCache();

		protected:

			void InnerDispose();

		public:

			vector<Vec3> vertices;

			struct Tri { unsigned int indices[3]; };
			vector<Tri> triangles;

			TriangleMeshShape();
			TriangleMeshShape(VertexBuffer* vbo);

			void DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori);

			vector<Intersection> RayTest(const Ray& ray);


			void Write(ostream& stream);
			unsigned int Read(istream& stream);
	};
}
