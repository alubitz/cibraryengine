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
			EntityList(const vector<Entity*>& entities);

			/** Returns how many entities are in this EntityList */
			unsigned int Count() const;
			/** Returns the Entity at the specified index within this EntityList */
			Entity* operator [](unsigned int index) const;

			/** Returns the subset of entities from this list which satisfy the specified condition */
			EntityList FindAll(EntityQualifier& p) const
			{
				EntityList result = EntityList();
				for(unsigned int i = 0; i < entities.size(); ++i)
					if(p.Accept(entities[i]))
						result.entities.push_back(entities[i]);
				return result;
			}

			/** Returns the boolean union of two sets of entities, i.e. a set containing all of the entities present in either of the inputs */
			static EntityList Union(const EntityList& a, const EntityList& b);
			/** Returns the boolean intersection of two sets of entities, i.e. a set containing only the entities which were present in both of the inputs */
			static EntityList Intersection(const EntityList& a, const EntityList& b);
			/** Returns the boolean difference of two sets of entities, i.e. the entities which were present in the first input and not present in the second */
			static EntityList Subtract(const EntityList& a, const EntityList& b);
	};
}
