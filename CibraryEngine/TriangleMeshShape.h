#pragma once

#include "StdAfx.h"
#include "CollisionShape.h"

#include "Plane.h"

#include "Octree.h"

#include "DebugLog.h"

namespace CibraryEngine
{
	struct Ray;
	struct Intersection;

	class TriangleMeshShape : public CollisionShape
	{
		public:

			struct TriCache
			{
				Vec3 a, b, c;
				Vec3 ab, ac;
				Vec3 p, q;
				Plane plane;
				float u_offset, v_offset;

				Vec3 bc;
				float inv_len_ab, inv_len_bc, inv_len_ca;
				Vec3 n_ab, n_bc, n_ca;
				float dot_n_ab_a, dot_n_bc_b, dot_n_ca_c;


				bool RayTest(const Ray& ray, unsigned int index, Intersection& intersection) const;
				float DistanceToPoint(const Vec3& point) const;
			};

		private:

			bool built;

			TriCache* cache;

			struct NodeData
			{
				struct TriRef
				{
					unsigned int face_index;
					AABB aabb;

					TriRef(unsigned int face_index, const AABB& aabb) : face_index(face_index), aabb(aabb) { }
				};

				vector<TriRef> triangles;
				unsigned int tri_count;

				NodeData() : triangles(), tri_count() { }

				void Add(const TriRef& tri) { triangles.push_back(tri); ++tri_count; }
			};

			struct RelevantTriangleGetter
			{
				set<unsigned int> relevant_list;				// list of unique maybe-relevant triangles encountered
				const AABB& aabb;

				RelevantTriangleGetter(const AABB& aabb);

				void operator() (Octree<NodeData>* node);
			};

			struct OctreeBuilder
			{
				const NodeData::TriRef& tri;

				OctreeBuilder(const NodeData::TriRef& tri);
				void operator()(Octree<NodeData>* node);
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

			void GetRelevantTriangles(const AABB& aabb, vector<unsigned int>& results);
			vector<Intersection> RayTest(const Ray& ray);

			const TriCache& GetTriangleData(unsigned int index) { return cache[index]; }

			AABB GetAABB();
			AABB GetTransformedAABB(const Mat4& xform);



			void Write(ostream& stream);
			unsigned int Read(istream& stream);
	};
}
