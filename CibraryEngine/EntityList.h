#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	using namespace std;

	class Entity;
	class EntityListIterator;

	/** Class representing some yes-or-no condition applicable to entities */
	class EntityQualifier
	{
		public:
			virtual bool Accept(Entity* entity) = 0;
	};

	/** Immutable class storing some collection of entities */
	class EntityList
	{
		private:

			vector<Entity*> entities;

		public:

			/** Initializes an empty EntityList */
			EntityList();
			/** Initializes an EntityList containing the specified entities */
			EntityList(vector<Entity*> entities);

			/** Returns how many entities are in this EntityList */
			unsigned int Count();
			/** Returns the Entity at the specified index within this EntityList */
			Entity* operator [](unsigned int index);

			/** Returns the subset of entities from this list which satisfy the specified condition */
			EntityList FindAll(EntityQualifier& p)
			{
				EntityList result = EntityList();
				for(unsigned int i = 0; i < entities.size(); i++)
					if(p.Accept(entities[i]))
						result.entities.push_back(entities[i]);
				return result;
			}

			/** Returns the boolean union of two sets of entities, i.e. a set containing all of the entities present in either of the inputs */
			static EntityList Union(EntityList& a, EntityList& b);
			/** Returns the boolean intersection of two sets of entities, i.e. a set containing only the entities which were present in both of the inputs */
			static EntityList Intersection(EntityList& a, EntityList& b);
			/** Returns the boolean difference of two sets of entities, i.e. the entities which were present in the first input and not present in the second */
			static EntityList Subtract(EntityList& a, EntityList& b);
	};
}
