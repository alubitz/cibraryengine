#include "StdAfx.h"
#include "Entity.h"
#include "TimingInfo.h"

using namespace CibraryEngine;

map<unsigned int, Entity*> all_entities = map<unsigned int, Entity*>();
unsigned long int next_id = 1;



Entity::Entity(GameState* gs) : id(next_id++), scripting_handle(NULL), game_state(gs), is_valid(true) { all_entities[id] = this; }
Entity::~Entity() { Dispose(); }

void Entity::Update(TimingInfo time) { }

void Entity::Vis(SceneRenderer* renderer) { }
void Entity::VisCleanup() { }

void Entity::InnerDispose()
{
	all_entities[id] = NULL;
	if(scripting_handle != NULL)
		*scripting_handle = NULL;
}

void Entity::Spawned() { }
void Entity::DeSpawned() { }

unsigned long int Entity::GetID() { return id; }

Entity** Entity::GetScriptingHandle()
{
	if(scripting_handle == NULL)
		scripting_handle = new Entity*(this);
	return scripting_handle;
}
