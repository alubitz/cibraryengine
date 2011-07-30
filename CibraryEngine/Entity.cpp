#include "StdAfx.h"
#include "Entity.h"
#include "TimingInfo.h"

using namespace CibraryEngine;

Entity::Entity(GameState* gs) : game_state(gs), is_valid(true) { }
Entity::~Entity() { }

void Entity::Update(TimingInfo time) { }

void Entity::Vis(SceneRenderer* renderer) { }
void Entity::VisCleanup() { }

void Entity::InnerDispose() { }

void Entity::Spawned() { }
void Entity::DeSpawned() { }
