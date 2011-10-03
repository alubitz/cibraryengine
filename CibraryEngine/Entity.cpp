#include "StdAfx.h"
#include "Entity.h"
#include "TimingInfo.h"

using namespace CibraryEngine;

Entity::Entity(GameState* gs) : game_state(gs), is_valid(true), scripting_handle(ScriptingHandle<Entity>::Default()) { }
Entity::~Entity() { Dispose(); }

void Entity::Update(TimingInfo time) { }

void Entity::Vis(SceneRenderer* renderer) { }
void Entity::VisCleanup() { }

void Entity::InnerDispose()
{
	scripting_handle.ObjectDeleted();
}

void Entity::Spawned() { }
void Entity::DeSpawned() { }

ScriptingHandle<Entity> Entity::GetScriptingHandle()
{
	if(!scripting_handle.HandleExists())
		scripting_handle = ScriptingHandle<Entity>::Wrap(this);

	return scripting_handle;
}
