#include "StdAfx.h"
#include "Physics.h"

#include "RigidBody.h"
#include "PhysicsRegion.h"

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

	/*
	 * PhysicsWorld private implementation struct
	 */
	struct PhysicsWorld::Imp
	{
		unordered_set<PhysicsRegion*> regions;
		PhysicsRegion* region;

		Vec3 gravity;

		float internal_timer, timer_interval;

		Imp() : regions(), gravity(0, -9.8f, 0), internal_timer(), timer_interval(1.0f / 60.0f) { region = new PhysicsRegion(); regions.insert(region); }
		~Imp()
		{
			// dispose rigid bodies
			for(unordered_set<PhysicsRegion*>::iterator iter = regions.begin(); iter != regions.end(); ++iter)
			{
				(*iter)->Dispose();
				delete *iter;
			}
			regions.clear();

			region = NULL;
		}

		void AddRigidBody(RigidBody* body)
		{
			region->AddRigidBody(body);
			body->gravity = gravity;					// set gravity upon adding to world
		}
		bool RemoveRigidBody(RigidBody* body) { return region->RemoveRigidBody(body); }

		// much cleaner than that quadratic formula nonsense
		void GetUseMass(const Vec3& direction, const ContactPoint& cp, float& A, float& B)
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

		// returns whether or not a collision response was actually needed (i.e. whether an impulse was applied to prevent interpenetration)
		bool DoCollisionResponse(const ContactPoint& cp)
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

		// do collision responses to make stuff work
		void SolveCollisionGraph(CollisionGraph& graph)
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

			// now go through each subgraph and do as many iterations as are necessary
			for(unordered_set<Subgraph*>::iterator iter = subgraphs.begin(); iter != subgraphs.end(); ++iter)
			{
				Subgraph& subgraph = **iter;

				for(int i = 0; i < MAX_SEQUENTIAL_SOLVER_ITERATIONS; ++i)
				{
					bool any = false;
					for(vector<ContactPoint*>::iterator jter = subgraph.contact_points.begin(); jter != subgraph.contact_points.end(); ++jter)
					{
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

			// clean up subgraphs
			for(unordered_set<Subgraph*>::iterator iter = subgraphs.begin(); iter != subgraphs.end(); ++iter)
				delete *iter;
		}

		void DoFixedStep()
		{
			float timestep = timer_interval;

			// set forces to what was applied by gravity / user forces
			for(unordered_set<PhysicsRegion*>::iterator iter = regions.begin(); iter != regions.end(); ++iter)
				(*iter)->UpdateVel(timestep);

			// handle all the collisions involving rays
			struct RayCallback : public CollisionCallback
			{
				Imp* imp;
				RayCallback(Imp* imp) : imp(imp) { }

				bool OnCollision(const ContactPoint& cp)			// return value controls whether to continue iterating through ray collisions
				{
					CollisionCallback* callback = cp.a.obj->GetCollisionCallback();

					if(callback && !callback->OnCollision(cp))
						return true;
					else
					{
						imp->DoCollisionResponse(cp);

						return true;
					}
				}
			} ray_callback(this);

			for(unordered_set<PhysicsRegion*>::iterator iter = regions.begin(); iter != regions.end(); ++iter)
				(*iter)->DoRayUpdates(timestep, ray_callback);

			// deal with all other collisions
			CollisionGraph collision_graph;

			for(unordered_set<PhysicsRegion*>::iterator iter = regions.begin(); iter != regions.end(); ++iter)
				(*iter)->AddCollisions(timestep, collision_graph);

			SolveCollisionGraph(collision_graph);

			// update positions
			for(unordered_set<PhysicsRegion*>::iterator iter = regions.begin(); iter != regions.end(); ++iter)
				(*iter)->UpdatePos(timestep);
		}

		void RayTest(const Vec3& from, const Vec3& to, CollisionCallback& callback, float max_time = 1.0f, RigidBody* ibody = NULL) { region->RayTest(from, to, callback, max_time, ibody); }
	};




	/*
	 * PhysicsWorld methods
	 */
	PhysicsWorld::PhysicsWorld() : imp(new Imp()) { }

	void PhysicsWorld::InnerDispose() { delete imp; imp = NULL; }

	void PhysicsWorld::AddRigidBody(RigidBody* r) { imp->AddRigidBody(r); }
	bool PhysicsWorld::RemoveRigidBody(RigidBody* r) { return imp->RemoveRigidBody(r); }

	void PhysicsWorld::Update(TimingInfo time)
	{
		imp->internal_timer += time.elapsed;
		while(imp->internal_timer >= imp->timer_interval)
		{
			imp->DoFixedStep();
			imp->internal_timer -= imp->timer_interval;
		}

		for(unordered_set<PhysicsRegion*>::iterator iter = imp->regions.begin(); iter != imp->regions.end(); ++iter)
			(*iter)->ResetForces();
	}

	void PhysicsWorld::DebugDrawWorld(SceneRenderer* renderer)
	{
		for(unordered_set<PhysicsRegion*>::iterator iter = imp->regions.begin(); iter != imp->regions.end(); ++iter)
			(*iter)->DebugDrawRegion(renderer);

		renderer->Render();
		renderer->Cleanup();
	}

	Vec3 PhysicsWorld::GetGravity() { return imp->gravity; }
	void PhysicsWorld::SetGravity(const Vec3& gravity)
	{
		imp->gravity = gravity;

		for(unordered_set<PhysicsRegion*>::iterator iter = imp->regions.begin(); iter != imp->regions.end(); ++iter)
			(*iter)->SetGravity(gravity);
	}

	void PhysicsWorld::RayTest(const Vec3& from, const Vec3& to, CollisionCallback& callback) { imp->RayTest(from, to, callback); }
}
