#include "StdAfx.h"
#include "Physics.h"

#include "RigidBody.h"
#include "PhysicsRegion.h"

#include "RayShape.h"
#include "SphereShape.h"
#include "TriangleMeshShape.h"
#include "InfinitePlaneShape.h"
#include "MultiSphereShape.h"

#include "CollisionGraph.h"

#include "Matrix.h"

#include "DebugLog.h"
#include "Serialize.h"

#include "RenderNode.h"
#include "SceneRenderer.h"

#include "Util.h"

#include "DebugDrawMaterial.h"

#define MAX_SEQUENTIAL_SOLVER_ITERATIONS 50

namespace CibraryEngine
{
	using boost::unordered_set;
	using boost::unordered_map;

	// forward declare a few functions
	static void DoRaySphere(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<RayResult>& hits);
	static void DoRayMesh(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<RayResult>& hits);
	static void DoRayPlane(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<RayResult>& hits);
	static void DoRayMultisphere(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<RayResult>& hits);

	static void DoSphereSphere(RigidBody* ibody, RigidBody* jbody, float radius, const Vec3& pos, const Vec3& vel, float timestep, CollisionGraph& hits);
	static void DoSphereMesh(RigidBody* ibody, RigidBody* jbody, float radius, const Vec3& pos, const Vec3& vel, float timestep, CollisionGraph& hits);
	static void DoSpherePlane(RigidBody* ibody, RigidBody* jbody, float radius, const Vec3& pos, const Vec3& vel, float timestep, CollisionGraph& hits);
	static void DoSphereMultisphere(RigidBody* ibody, RigidBody* jbody, float radius, const Vec3& pos, const Vec3& vel, float timestep, CollisionGraph& hits);

	static void DoMultisphereMesh(RigidBody* ibody, RigidBody* jbody, MultiSphereShape* ishape, CollisionGraph& hits);
	static void DoMultispherePlane(RigidBody* ibody, RigidBody* jbody, MultiSphereShape* ishape, const Mat4& xform, CollisionGraph& hits);
	static void DoMultisphereMultisphere(RigidBody* ibody, RigidBody* jbody, MultiSphereShape* ishape, const Mat4& xform, CollisionGraph& hits);




	/*
	 * PhysicsWorld orphan callback struct (private)
	 */
	struct PhysicsWorld::MyOrphanCallback : public ObjectOrphanedCallback
	{
		bool shutdown;

		MyOrphanCallback() : shutdown(false) { }

		void OnObjectOrphaned(RigidBody* object)
		{
			//if(!shutdown)
			//	Debug("An object has been orphaned!\n");
		}
	};




	/*
	 * GridRegionManager, a subclass of PhysicsRegionManager
	 */
	class GridRegionManager : public PhysicsRegionManager
	{
		public:

			ObjectOrphanedCallback* orphan_callback;

			// size of each cell
			float cell_dim;

			// offset of the minimum cell
			int x0, y0, z0;
			int dx, dy, dz;

			vector<vector<vector<PhysicsRegion*> > > region_array;
			unordered_set<RigidBody*> planes;

			GridRegionManager(unordered_set<PhysicsRegion*>* all_regions, ObjectOrphanedCallback* orphan_callback) :
				PhysicsRegionManager(all_regions),
				orphan_callback(orphan_callback),
				cell_dim(16.0f),
				x0(-8),
				y0(-2),
				z0(-8),
				dx(16),
				dy(22),
				dz(16),
				region_array(),
				planes()
			{
				for(int x = 0; x < dx; ++x)
				{
					region_array.push_back(vector<vector<PhysicsRegion*> >());
					for(int y = 0; y < dy; ++y)
					{
						region_array[x].push_back(vector<PhysicsRegion*>());
						for(int z = 0; z < dz; ++z)
						{
							region_array[x][y].push_back(CreateRegion(x + x0, y + y0, z + z0));
						}
					}
				}
			}

			~GridRegionManager() { }

			void AABBToCells(const AABB& aabb, int& x1, int& y1, int& z1, int& x2, int& y2, int& z2)
			{
				x1 = (int)floor(aabb.min.x / cell_dim);
				y1 = (int)floor(aabb.min.y / cell_dim);
				z1 = (int)floor(aabb.min.z / cell_dim);
				x2 = (int)ceil(aabb.max.x / cell_dim);
				y2 = (int)ceil(aabb.max.y / cell_dim);
				z2 = (int)ceil(aabb.max.z / cell_dim);
			}

			PhysicsRegion* CreateRegion(int x, int y, int z)
			{
				PhysicsRegion* result = new PhysicsRegion(orphan_callback);

				all_regions->insert(result);

				for(unordered_set<RigidBody*>::iterator iter = planes.begin(); iter != planes.end(); ++iter)
					result->TakeOwnership(*iter);

				return result;
			}

			void OnObjectAdded(RigidBody* object, set<PhysicsRegion*>& object_regions)
			{
				if(object->GetCollisionShape()->GetShapeType() != ST_InfinitePlane)
				{
					int x1, y1, z1, x2, y2, z2;
					AABBToCells(object->GetAABB(0), x1, y1, z1, x2, y2, z2);

					x1 = max(x0, x1);
					y1 = max(y0, y1);
					z1 = max(z0, z1);
					x2 = min(x0 + dx - 1, x2);
					y2 = min(y0 + dy - 1, y2);
					z2 = min(z0 + dz - 1, z2);
				
					for(int x = x1; x < x2; ++x)
						for(int y = y1; y < y2; ++y)
							for(int z = z1; z < z2; ++z)
								region_array[x - x0][y - y0][z - z0]->TakeOwnership(object);
				}
				else
				{
					for(unsigned int x = 0; x < region_array.size(); ++x)
						for(unsigned int y = 0; y < region_array[x].size(); ++y)
							for(unsigned int z = 0; z < region_array[x][y].size(); ++z)
								region_array[x][y][z]->TakeOwnership(object);

					planes.insert(object);
				}
			}

			void OnObjectUpdate(RigidBody* object, set<PhysicsRegion*>& object_regions, float timestep)
			{
				for(set<PhysicsRegion*>::iterator iter = object_regions.begin(); iter != object_regions.end(); ++iter)
				{
					PhysicsRegion* region = *iter;
					region->RemoveRigidBody(object);
				}

				object_regions.clear();

				OnObjectAdded(object, object_regions);

				if(object_regions.empty() && orphan_callback)
					orphan_callback->OnObjectOrphaned(object);
			}

			void OnObjectRemoved(RigidBody* object, set<PhysicsRegion*>& object_regions)
			{
				for(set<PhysicsRegion*>::iterator iter = object_regions.begin(); iter != object_regions.end(); ++iter)
				{
					PhysicsRegion* region = *iter;
					region->RemoveRigidBody(object);
				}

				if(object->GetCollisionShape()->GetShapeType() == ST_InfinitePlane)
					planes.erase(object);
			}

			PhysicsRegion* GetRegion(const Vec3& point)
			{
				int x1, y1, z1, x2, y2, z2;
				AABBToCells(AABB(point), x1, y1, z1, x2, y2, z2);

				return region_array[x1 - x0][y1 - y0][z1 - z0];
			}

			void GetRegionsOnRay(const Vec3& from, const Vec3& to, set<PhysicsRegion*>& results)
			{
				AABB aabb(from);
				aabb.Expand(to);

				int x1, y1, z1, x2, y2, z2;
				AABBToCells(aabb, x1, y1, z1, x2, y2, z2);

				Vec3 cell_diagonal(cell_dim, cell_dim, cell_dim);

				x1 = max(x0, x1);
				y1 = max(y0, y1);
				z1 = max(z0, z1);
				x2 = min(x0 + dx - 1, x2);
				y2 = min(y0 + dy - 1, y2);
				z2 = min(z0 + dz - 1, z2);

				for(int x = x1; x < x2; ++x)
					for(int y = y1; y < y2; ++y)
						for(int z = z1; z < z2; ++z)
						{
							Vec3 xyz(x * cell_dim, y * cell_dim, z * cell_dim);

							if(AABB(xyz, xyz + cell_diagonal).IntersectLineSegment(from, to))
								results.insert(region_array[x - x0][y - y0][z - z0]);
						}
			}
	};




	/*
	 * PhysicsWorld methods
	 */
	PhysicsWorld::PhysicsWorld() :
		all_objects(),
		dynamic_objects(),
		all_regions(),
		region_man(NULL),
		gravity(0, -9.8f, 0),
		internal_timer(),
		timer_interval(1.0f / 60.0f),
		orphan_callback(new MyOrphanCallback())
	{
		region_man = new GridRegionManager(&all_regions, orphan_callback);
	}

	void PhysicsWorld::InnerDispose()
	{
		// suppress "object orphaned" messages
		orphan_callback->shutdown = true;

		// dispose physics regions
		for(unordered_set<PhysicsRegion*>::iterator iter = all_regions.begin(); iter != all_regions.end(); ++iter)
		{
			(*iter)->Dispose();
			delete *iter;
		}
		all_regions.clear();

		// dispose rigid bodies
		for(unsigned int i = 0; i < ST_ShapeTypeMax; ++i)
		{
			for(unordered_set<RigidBody*>::iterator iter = all_objects[i].begin(); iter != all_objects[i].end(); ++iter)
			{
				(*iter)->Dispose();
				delete *iter;
			}

			all_objects[i].clear();
			dynamic_objects[i].clear();
		}

		delete region_man;
		region_man = NULL;

		delete orphan_callback;
		orphan_callback = NULL;
	}



	void PhysicsWorld::GetUseMass(const Vec3& direction, const ContactPoint& cp, float& A, float& B)
	{
		RigidBody* ibody = cp.a.obj;
		RigidBody* jbody = cp.b.obj;

		float m1 = ibody->mass_info.mass, m2 = jbody->mass_info.mass;

		A = B = 0;

		if(jbody->can_move)
		{
			A = ibody->inv_mass + jbody->inv_mass;

			Vec3 i_lvel = ibody->vel, j_lvel = jbody->vel;
			B = Vec3::Dot(i_lvel, direction) - Vec3::Dot(j_lvel, direction);
		}
		else
		{
			A = ibody->inv_mass;
			B = Vec3::Dot(ibody->vel, direction);
		}

		if(ibody->can_rotate)
		{
			Vec3 nr1 = Vec3::Cross(direction, cp.a.pos - ibody->GetTransformationMatrix().TransformVec3(ibody->mass_info.com, 1.0f));
			A += Vec3::Dot(ibody->inv_moi * nr1, nr1);
			B += Vec3::Dot(ibody->rot, nr1);
		}

		if(jbody->can_rotate)
		{
			Vec3 nr2 = Vec3::Cross(direction, cp.b.pos - jbody->GetTransformationMatrix().TransformVec3(jbody->mass_info.com, 1.0f));
			A += Vec3::Dot(jbody->inv_moi * nr2, nr2);
			B -= Vec3::Dot(jbody->rot, nr2);
		}
	}

	bool PhysicsWorld::DoCollisionResponse(const ContactPoint& cp)
	{
		RigidBody* ibody = cp.a.obj;
		RigidBody* jbody = cp.b.obj;

		bool j_can_move = jbody->can_move;

		float m1 = ibody->mass_info.mass;
		float m2 = jbody->mass_info.mass;

		if(m1 + m2 > 0)
		{
			Vec3 i_poi = ibody->GetInvTransform().TransformVec3(cp.a.pos, 1.0f);
			Vec3 j_poi = jbody->GetInvTransform().TransformVec3(cp.b.pos, 1.0f);

			Vec3 i_v = ibody->GetLocalVelocity(cp.a.pos);
			Vec3 j_v = jbody->GetLocalVelocity(cp.b.pos);
					
			Vec3 dv = j_v - i_v;
			const Vec3& normal = Vec3::Normalize(cp.a.norm - cp.b.norm);

			float nvdot = Vec3::Dot(normal, dv);
			if(nvdot < 0.0f)
			{
				float A, B;
				GetUseMass(normal, cp, A, B);

				float use_mass = 1.0f / A;
				float bounciness = ibody->bounciness * jbody->bounciness;
				float impulse_mag = -(1.0f + bounciness) * B * use_mass;
				
				if(impulse_mag < 0)
				{
					Vec3 impulse = normal * impulse_mag;

					if(impulse.ComputeMagnitudeSquared() != 0)
					{
						ibody->ApplyImpulse(impulse, i_poi);
						if(j_can_move)
							jbody->ApplyImpulse(-impulse, j_poi);

						// applying this impulse means we need to recompute dv and nvdot!
						dv = jbody->GetLocalVelocity(cp.b.pos) - ibody->GetLocalVelocity(cp.a.pos);
						nvdot = Vec3::Dot(normal, dv);
					}

					float sfric_coeff = ibody->friction * jbody->friction;
					float kfric_coeff = 0.9f * sfric_coeff;

					Vec3 t_dv = dv - normal * nvdot;
					float t_dv_magsq = t_dv.ComputeMagnitudeSquared();

					if(t_dv_magsq > 0.001f)							// object is moving; apply kinetic friction
					{
						float t_dv_mag = sqrtf(t_dv_magsq), inv_tdmag = 1.0f / t_dv_mag;
						Vec3 u_tdv = t_dv * inv_tdmag;

						GetUseMass(u_tdv, cp, A, B);
						use_mass = 1.0f / A;

						Vec3 fric_impulse = t_dv * min(use_mass, fabs(impulse_mag * kfric_coeff * inv_tdmag));

						ibody->ApplyImpulse(fric_impulse, i_poi);
						if(j_can_move)
							jbody->ApplyImpulse(-fric_impulse, j_poi);
					}
					else											// object isn't moving; apply static friction
					{
						Vec3 df = jbody->applied_force - ibody->applied_force;
						float nfdot = Vec3::Dot(normal, df);

						Vec3 t_df = df - normal * nfdot;
						float t_df_mag = t_df.ComputeMagnitude();

						float fric_i_mag = min(impulse_mag * sfric_coeff, t_df_mag);
						if(fric_i_mag > 0)
						{
							Vec3 fric_impulse = t_df * (-fric_i_mag / t_df_mag);

							ibody->ApplyImpulse(fric_impulse, i_poi);
							if(j_can_move)
								jbody->ApplyImpulse(-fric_impulse, j_poi);
						}
					}

					return true;
				}
			}
		}

		return false;
	}

	void PhysicsWorld::SolveCollisionGraph(CollisionGraph& graph)
	{
		// break the graph into separate subgraphs
		struct Subgraph
		{
			unordered_set<CollisionGraph::Node*> nodes;
			vector<ContactPoint*> contact_points;

			Subgraph() : contact_points() { }

			bool ContainsNode(CollisionGraph::Node* node) { return nodes.find(node) != nodes.end(); }
		};
		
		unordered_set<Subgraph*> subgraphs;
		unordered_map<RigidBody*, Subgraph*> body_subgraphs;
		
		for(map<RigidBody*, CollisionGraph::Node*>::iterator iter = graph.nodes.begin(); iter != graph.nodes.end(); ++iter)
		{
			unordered_map<RigidBody*, Subgraph*>::iterator found = body_subgraphs.find(iter->first);
			
			if(found == body_subgraphs.end())
			{
				Subgraph* subgraph = new Subgraph();
				subgraphs.insert(subgraph);

				vector<CollisionGraph::Node*> fringe;
				fringe.push_back(iter->second);

				while(!fringe.empty())
				{
					CollisionGraph::Node* node = *fringe.rbegin();
					subgraph->nodes.insert(node);

					body_subgraphs[node->body] = subgraph;

					fringe.pop_back();

					for(vector<CollisionGraph::Edge>::iterator jter = node->edges.begin(); jter != node->edges.end(); ++jter)
					{
						subgraph->contact_points.push_back(jter->cp);

						if(CollisionGraph::Node* other = jter->other_node)
							if(!subgraph->ContainsNode(other))
								fringe.push_back(other);
					}
				}
			}
		}

		//int count = 0;

		// now go through each subgraph and do as many iterations as are necessary
		for(unordered_set<Subgraph*>::iterator iter = subgraphs.begin(); iter != subgraphs.end(); ++iter)
		{
			Subgraph& subgraph = **iter;

			for(int i = 0; i < MAX_SEQUENTIAL_SOLVER_ITERATIONS; ++i)
			{
				bool any = false;
				for(vector<ContactPoint*>::iterator jter = subgraph.contact_points.begin(); jter != subgraph.contact_points.end(); ++jter)
				{
					//++count;
					if(DoCollisionResponse(**jter))
					{
						any = true;

						if((*jter)->a.obj->collision_callback)
							(*jter)->a.obj->collision_callback->OnCollision(**jter);

						if((*jter)->b.obj->collision_callback)
							(*jter)->b.obj->collision_callback->OnCollision(**jter);
					}
				}

				if(!any)
					break;
			}
		}

		//Debug(((stringstream&)(stringstream() << "count = " << count << "; collision graph contains " << graph.contact_points.size() << " contact points, " << graph.nodes.size() << " nodes, and " << subgraphs.size() << " subgraphs" << endl)).str());

		// clean up subgraphs
		for(unordered_set<Subgraph*>::iterator iter = subgraphs.begin(); iter != subgraphs.end(); ++iter)
			delete *iter;
	}

	void PhysicsWorld::InitiateCollisionsForSphere(RigidBody* body, float timestep, CollisionGraph& collision_graph) 
	{
		SphereShape* shape = (SphereShape*)body->GetCollisionShape();

		float radius = shape->radius;
		Vec3 pos = body->GetPosition();
		Vec3 vel = body->GetLinearVelocity();

		// find out what might be colliding with us
		AABB aabb(pos, radius);
		aabb.Expand(AABB(pos + vel, radius));

		unordered_set<RigidBody*> relevant_objects[ST_ShapeTypeMax];
		for(set<PhysicsRegion*>::iterator iter = body->regions.begin(); iter != body->regions.end(); ++iter)
			(*iter)->GetRelevantObjects(aabb, relevant_objects);

		// do collision detection on those objects
		for(unordered_set<RigidBody*>::iterator iter = relevant_objects[ST_Sphere].begin(); iter != relevant_objects[ST_Sphere].end(); ++iter)
			if(*iter < body)
				DoSphereSphere(body, *iter, radius, pos, vel, timestep, collision_graph);

		for(unordered_set<RigidBody*>::iterator iter = relevant_objects[ST_TriangleMesh].begin(); iter != relevant_objects[ST_TriangleMesh].end(); ++iter)
			DoSphereMesh(body, *iter, radius, pos, vel, timestep, collision_graph);

		for(unordered_set<RigidBody*>::iterator iter = relevant_objects[ST_InfinitePlane].begin(); iter != relevant_objects[ST_InfinitePlane].end(); ++iter)
			DoSpherePlane(body, *iter, radius, pos, vel, timestep, collision_graph);

		for(unordered_set<RigidBody*>::iterator iter = relevant_objects[ST_MultiSphere].begin(); iter != relevant_objects[ST_MultiSphere].end(); ++iter)
			DoSphereMultisphere(body, *iter, radius, pos, vel, timestep, collision_graph);
	}

	void PhysicsWorld::InitiateCollisionsForMultiSphere(RigidBody* body, float timestep, CollisionGraph& collision_graph) 
	{
		MultiSphereShape* shape = (MultiSphereShape*)body->GetCollisionShape();

		Mat4 xform = body->GetTransformationMatrix();
		AABB xformed_aabb = shape->GetTransformedAABB(xform);

		// find out what might be colliding with us
		unordered_set<RigidBody*> relevant_objects[ST_ShapeTypeMax];
		for(set<PhysicsRegion*>::iterator iter = body->regions.begin(); iter != body->regions.end(); ++iter)
			(*iter)->GetRelevantObjects(xformed_aabb, relevant_objects);

		// do collision detection on those objects
		for(unordered_set<RigidBody*>::iterator iter = relevant_objects[ST_TriangleMesh].begin(); iter != relevant_objects[ST_TriangleMesh].end(); ++iter)
			DoMultisphereMesh(body, *iter, shape, collision_graph);

		for(unordered_set<RigidBody*>::iterator iter = relevant_objects[ST_InfinitePlane].begin(); iter != relevant_objects[ST_InfinitePlane].end(); ++iter)
			DoMultispherePlane(body, *iter, shape, xform, collision_graph);
		
		for(unordered_set<RigidBody*>::iterator iter = relevant_objects[ST_MultiSphere].begin(); iter != relevant_objects[ST_MultiSphere].end(); ++iter)
			if(*iter < body)
				DoMultisphereMultisphere(body, *iter, shape, xform, collision_graph);
	}

	void PhysicsWorld::DoFixedStep()
	{
		float timestep = timer_interval;

		// set forces to what was applied by gravity / user forces; also the objects may deactivate or move between physics regions now
		for(unsigned int i = 1; i < ST_ShapeTypeMax; ++i)
			if(CollisionShape::CanShapeTypeMove((ShapeType)i))
				for(unordered_set<RigidBody*>::iterator iter = dynamic_objects[i].begin(); iter != dynamic_objects[i].end(); ++iter)
					(*iter)->UpdateVel(timestep);

		// handle all the collisions involving rays
		struct RayCallback : public CollisionCallback
		{
			PhysicsWorld* world;
			RayCallback(PhysicsWorld* world) : world(world) { }

			bool OnCollision(const ContactPoint& cp)			// return value controls whether to continue iterating through ray collisions
			{
				CollisionCallback* callback = cp.a.obj->GetCollisionCallback();

				if(callback && !callback->OnCollision(cp))
					return true;
				else
				{
					world->DoCollisionResponse(cp);

					return true;
				}
			}
		} ray_callback(this);

		for(unordered_set<RigidBody*>::iterator iter = dynamic_objects[ST_Ray].begin(); iter != dynamic_objects[ST_Ray].end(); ++iter)
		{
			RigidBody* body = *iter;
			RayTestPrivate(body->pos, body->pos + body->vel, ray_callback, timestep, body);
		}

		// populate a collision graph with all the collisions that are going on
		CollisionGraph collision_graph;

		for(unordered_set<RigidBody*>::iterator iter = dynamic_objects[ST_Sphere].begin(); iter != dynamic_objects[ST_Sphere].end(); ++iter)
			InitiateCollisionsForSphere(*iter, timestep, collision_graph);

		for(unordered_set<RigidBody*>::iterator iter = dynamic_objects[ST_MultiSphere].begin(); iter != dynamic_objects[ST_MultiSphere].end(); ++iter)
			InitiateCollisionsForMultiSphere(*iter, timestep, collision_graph);	

		SolveCollisionGraph(collision_graph);

		// update positions
		for(unsigned int i = 1; i < ST_ShapeTypeMax; ++i)
			if(CollisionShape::CanShapeTypeMove((ShapeType)i))
				for(unordered_set<RigidBody*>::iterator iter = dynamic_objects[i].begin(); iter != dynamic_objects[i].end(); ++iter)
					(*iter)->UpdatePos(timestep, region_man);
	}



	void PhysicsWorld::AddRigidBody(RigidBody* r)
	{
		ShapeType type = r->GetCollisionShape()->GetShapeType();

		all_objects[type].insert(r);
		if(r->can_move)
			dynamic_objects[type].insert(r);

		r->gravity = gravity;

		region_man->OnObjectAdded(r, r->regions);
	}

	void PhysicsWorld::RemoveRigidBody(RigidBody* r)
	{
		ShapeType type = r->GetCollisionShape()->GetShapeType();
		all_objects[type].erase(r);
		if(r->can_move)
			dynamic_objects[type].erase(r);

		region_man->OnObjectRemoved(r, r->regions);

		const set<PhysicsRegion*>& regions = r->regions;
		for(set<PhysicsRegion*>::const_iterator iter = regions.begin(); iter != regions.end(); ++iter)
			(*iter)->RemoveRigidBody(r);
		r->regions.clear();
	}

	void PhysicsWorld::Update(TimingInfo time)
	{
		internal_timer += time.elapsed;
		while(internal_timer >= timer_interval)
		{
			DoFixedStep();
			internal_timer -= timer_interval;
		}

		for(unsigned int i = 1; i < ST_ShapeTypeMax; ++i)
			if(CollisionShape::CanShapeTypeMove((ShapeType)i))
				for(unordered_set<RigidBody*>::iterator iter = all_objects[i].begin(); iter != all_objects[i].end(); ++iter)
					(*iter)->ResetForces();
	}

	void PhysicsWorld::DebugDrawWorld(SceneRenderer* renderer)
	{
		for(unsigned int i = 0; i < ST_ShapeTypeMax; ++i)
			for(unordered_set<RigidBody*>::iterator iter = all_objects[i].begin(); iter != all_objects[i].end(); ++iter)
				(*iter)->DebugDraw(renderer);

		renderer->Render();
		renderer->Cleanup();
	}

	Vec3 PhysicsWorld::GetGravity() { return gravity; }
	void PhysicsWorld::SetGravity(const Vec3& gravity_)
	{
		gravity = gravity_;

		for(unsigned int i = 1; i < ST_ShapeTypeMax; ++i)
			if(CollisionShape::CanShapeTypeMove((ShapeType)i))
				for(unordered_set<RigidBody*>::iterator iter = all_objects[i].begin(); iter != all_objects[i].end(); ++iter)
					(*iter)->gravity = gravity_;
	}

	void PhysicsWorld::RayTestPrivate(const Vec3& from, const Vec3& to, CollisionCallback& callback, float max_time, RigidBody* ibody)
	{
		// find out what objects are relevant
		set<PhysicsRegion*> regions;
		Vec3 endpoint = from + (to - from) * max_time;

		region_man->GetRegionsOnRay(from, endpoint, regions);

		AABB ray_aabb(from);
		ray_aabb.Expand(endpoint);

		unordered_set<RigidBody*> relevant_objects[ST_ShapeTypeMax];
		for(set<PhysicsRegion*>::iterator iter = regions.begin(); iter != regions.end(); ++iter)
			(*iter)->GetRelevantObjects(ray_aabb, relevant_objects);

		// now do the actual collision testing
		Ray ray;
		ray.origin = from;
		ray.direction = to - from;

		list<RayResult> hits;
		
		for(unordered_set<RigidBody*>::iterator jter = relevant_objects[ST_Sphere].begin(); jter != relevant_objects[ST_Sphere].end(); ++jter)
			DoRaySphere(ibody, *jter, ray, max_time, hits);

		for(unordered_set<RigidBody*>::iterator jter = relevant_objects[ST_TriangleMesh].begin(); jter != relevant_objects[ST_TriangleMesh].end(); ++jter)
			DoRayMesh(ibody, *jter, ray, max_time, hits);

		for(unordered_set<RigidBody*>::iterator jter = relevant_objects[ST_InfinitePlane].begin(); jter != relevant_objects[ST_InfinitePlane].end(); ++jter)
			DoRayPlane(ibody, *jter, ray, max_time, hits);

		for(unordered_set<RigidBody*>::iterator jter = relevant_objects[ST_MultiSphere].begin(); jter != relevant_objects[ST_MultiSphere].end(); ++jter)
			DoRayMultisphere(ibody, *jter, ray, max_time, hits);

		// run the collision callback on whatever we found
		if(!hits.empty())
		{
			hits.sort();

			for(list<RayResult>::iterator jter = hits.begin(); jter != hits.end(); ++jter)
				if(callback.OnCollision(jter->p))
					break;
		}
	}
	void PhysicsWorld::RayTest(const Vec3& from, const Vec3& to, CollisionCallback& callback) { RayTestPrivate(from, to, callback); }




	/*
	 * Ray intersect functions
	 */
	static void DoRaySphere(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<RayResult>& hits)
	{
		float first, second;

		if(Util::RaySphereIntersect(ray, Sphere(jbody->GetPosition(), ((SphereShape*)jbody->GetCollisionShape())->radius), first, second))
			if(first >= 0 && first < max_time)
			{
				ContactPoint p;

				p.a.obj = ibody;
				p.b.obj = jbody;
				p.a.pos = ray.origin + first * ray.direction;
				p.b.norm = Vec3::Normalize(p.a.pos - jbody->GetPosition(), 1.0f);

				hits.push_back(RayResult(first, p));
			}
	}

	static void DoRayMesh(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<RayResult>& hits)
	{
		TriangleMeshShape* mesh = (TriangleMeshShape*)jbody->GetCollisionShape();

		Mat4 inv_mat = jbody->GetInvTransform();

		Ray ray_cut;
		ray_cut.origin = inv_mat.TransformVec3(ray.origin, 1.0f);
		ray_cut.direction = inv_mat.TransformVec3(ray.direction * max_time, 0.0f);

		vector<Intersection> mesh_hits = mesh->RayTest(ray_cut);

		for(vector<Intersection>::iterator kter = mesh_hits.begin(); kter != mesh_hits.end(); ++kter)
		{
			float t = kter->time * max_time;
			if(t >= 0 && t < max_time)
			{
				ContactPoint p;

				p.a.obj = ibody;
				p.b.obj = jbody;
				p.a.pos = ray.origin + t * ray.direction;
				p.b.norm = kter->normal;

				hits.push_back(RayResult(t, p));
			}
		}
	}

	static void DoRayPlane(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<RayResult>& hits)
	{
		InfinitePlaneShape* plane = (InfinitePlaneShape*)jbody->GetCollisionShape();

		float t = Util::RayPlaneIntersect(ray, plane->plane);
		if(t >= 0 && t < max_time)
		{
			ContactPoint p;

			p.a.obj = ibody;
			p.b.obj = jbody;
			p.a.pos = ray.origin + t * ray.direction;
			p.b.norm = plane->plane.normal;

			hits.push_back(RayResult(t, p));
		}
	}

	static void DoRayMultisphere(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<RayResult>& hits)
	{
		MultiSphereShape* shape = (MultiSphereShape*)jbody->GetCollisionShape();

		Mat4 inv_mat = jbody->GetInvTransform();

		Ray nu_ray;
		nu_ray.origin = inv_mat.TransformVec3(ray.origin, 1.0f);
		nu_ray.direction = inv_mat.TransformVec3(ray.direction * max_time, 0.0f);

		ContactPoint p;
		float t;
		if(shape->CollisionCheck(nu_ray, p, t, ibody, jbody))
		{
			p.a.pos = jbody->GetTransformationMatrix().TransformVec3(p.b.pos, 1.0f);
			hits.push_back(RayResult(t * max_time, p));
		}
	}




	/*
	 * SphereShape collision functions
	 */
	static void DoSphereSphere(RigidBody* ibody, RigidBody* jbody, float radius, const Vec3& pos, const Vec3& vel, float timestep, CollisionGraph& hits)
	{
		float sr = radius + ((SphereShape*)jbody->GetCollisionShape())->radius;

		Vec3 other_pos = jbody->GetPosition();
		Vec3 other_vel = jbody->GetLinearVelocity();

		Ray ray;
		ray.origin = pos - other_pos;
		ray.direction = vel - other_vel;

		float first = 0, second = 0;
		if(ray.origin.ComputeMagnitudeSquared() < sr * sr || Util::RaySphereIntersect(ray, Sphere(Vec3(), sr), first, second))
			if(first >= 0 && first <= timestep)
			{
				ContactPoint p;

				p.a.obj = ibody;
				p.b.obj = jbody;
				p.a.pos = pos + first * vel;
				p.b.pos = other_pos + first * other_vel;
				p.b.norm = Vec3::Normalize(p.a.pos - other_pos, 1.0f);
				p.a.norm = -p.b.norm;

				hits.AddContactPoint(p);
			}
	}

	static void DoSphereMesh(RigidBody* ibody, RigidBody* jbody, float radius, const Vec3& pos, const Vec3& vel, float timestep, CollisionGraph& hits)
	{
		TriangleMeshShape* shape = (TriangleMeshShape*)jbody->GetCollisionShape();

		Ray ray;
		ray.origin = pos;
		ray.direction = vel;

		vector<unsigned int> relevant_triangles = shape->GetRelevantTriangles(AABB(pos, radius));
		for(vector<unsigned int>::iterator kter = relevant_triangles.begin(); kter != relevant_triangles.end(); ++kter)
		{
			TriangleMeshShape::TriCache tri = shape->GetTriangleData(*kter);
							
			float dist = tri.DistanceToPoint(pos);
			if(dist < radius)
			{
				ContactPoint p;

				p.a.obj = ibody;
				p.b.obj = jbody;
				p.a.pos = pos - tri.plane.normal * radius;
				p.b.pos = p.a.pos;
				p.a.norm = -tri.plane.normal;
				p.b.norm = tri.plane.normal;

				hits.AddContactPoint(p);
			}
		}
	}

	static void DoSpherePlane(RigidBody* ibody, RigidBody* jbody, float radius, const Vec3& pos, const Vec3& vel, float timestep, CollisionGraph& hits)
	{
		InfinitePlaneShape* shape = (InfinitePlaneShape*)jbody->GetCollisionShape();

		Vec3 plane_norm = shape->plane.normal;
		float plane_offset = shape->plane.offset;

		float center_offset = Vec3::Dot(plane_norm, pos) - plane_offset;
		float vel_dot = Vec3::Dot(plane_norm, vel);

		// if the sphere is moving away from the plane, don't bounce!
		if(vel_dot <= 0.0f)
		{
			float dist = fabs(center_offset) - radius;
			float t = center_offset - radius < 0.0f ? 0.0f : dist / fabs(vel_dot);
			if(t >= 0 && t <= timestep)
			{
				ContactPoint p;

				p.a.obj = ibody;
				p.b.obj = jbody;
				p.a.pos = pos - plane_norm * radius;
				p.b.pos = p.a.pos - plane_norm * (Vec3::Dot(p.a.pos, plane_norm) - plane_offset);
				p.b.norm = plane_norm;
				p.a.norm = -plane_norm;

				hits.AddContactPoint(p);
			}
		}
	}

	static void DoSphereMultisphere(RigidBody* ibody, RigidBody* jbody, float radius, const Vec3& pos, const Vec3& vel, float timestep, CollisionGraph& hits)
	{
		MultiSphereShape* shape = (MultiSphereShape*)jbody->GetCollisionShape();

		Mat4 inv_xform = Mat4::Invert(jbody->GetTransformationMatrix());
		Vec3 nu_pos = inv_xform.TransformVec3(pos, 1.0f);

		ContactPoint cp;
		if(shape->CollisionCheck(Sphere(nu_pos, radius), cp, ibody, jbody))
			hits.AddContactPoint(cp);
	}




	/*
	 * MultiSphereShape collision functions
	 */
	static void DoMultisphereMesh(RigidBody* ibody, RigidBody* jbody, MultiSphereShape* ishape, CollisionGraph& hits)
	{
		TriangleMeshShape* jshape = (TriangleMeshShape*)jbody->GetCollisionShape();

		Mat4 inv_net_xform = jbody->GetInvTransform() * ibody->GetTransformationMatrix();
		AABB xformed_aabb = ishape->GetTransformedAABB(inv_net_xform);						// the AABB of the multisphere in the coordinate system of the mesh

		vector<unsigned int> relevant_triangles = jshape->GetRelevantTriangles(xformed_aabb);
		if(relevant_triangles.empty())
			return;

		ContactPoint p;
		for(vector<unsigned int>::iterator kter = relevant_triangles.begin(); kter != relevant_triangles.end(); ++kter)
		{
			TriangleMeshShape::TriCache tri = jshape->GetTriangleData(*kter);

			ContactPoint p;
			if(ishape->CollisionCheck(inv_net_xform, tri, p, ibody, jbody))
			{
				Mat4 j_xform = jbody->GetTransformationMatrix();

				p.a.pos = p.b.pos = j_xform.TransformVec3(p.a.pos, 1.0f);
				p.a.norm = j_xform.TransformVec3(p.a.norm, 0.0f);
				p.b.norm = -p.a.norm;

				hits.AddContactPoint(p);
			}
		}
	}

	static void DoMultispherePlane(RigidBody* ibody, RigidBody* jbody, MultiSphereShape* ishape, const Mat4& xform, CollisionGraph& hits)
	{
		InfinitePlaneShape* jshape = (InfinitePlaneShape*)jbody->GetCollisionShape();

		ContactPoint p;
		if(ishape->CollisionCheck(xform, jshape->plane, p, ibody, jbody))
			hits.AddContactPoint(p);
	}

	static void DoMultisphereMultisphere(RigidBody* ibody, RigidBody* jbody, MultiSphereShape* ishape, const Mat4& xform, CollisionGraph& hits)
	{
		MultiSphereShape* jshape = (MultiSphereShape*)jbody->GetCollisionShape();

		Mat4 net_xform = ibody->GetInvTransform() * jbody->GetTransformationMatrix();

		ContactPoint p;
		if(ishape->CollisionCheck(net_xform, jshape, p, ibody, jbody))
		{
			p.a.pos = p.b.pos = xform.TransformVec3(p.a.pos, 1.0f);
			p.a.norm = xform.TransformVec3(p.a.norm, 0.0f);
			p.b.norm = -p.a.norm;

			hits.AddContactPoint(p);
		}
	}
}
