#pragma once

#include "StdAfx.h"
#include "Entity.h"

namespace CibraryEngine
{
	using namespace std;

	class Controller;

	/** The collection of controls for a Pawn */
	class ControlState
	{
		public:

			/** Initializes a control state */
			ControlState();
			~ControlState();
	};

	/** An entity which is being controlled */
	class Pawn : public Entity
	{
		public:

			/** The thing doing the controlling; this could be an AI, or a player */
			Controller* controller;
			/** The current state of the controls for this Pawn */
			ControlState* control_state;

			/** Initializes a Pawn for the specified GameState */
			Pawn(GameState* gs);
			virtual ~Pawn();
	};

	/** Class representing something which can control a Pawn; a Controller is also an Entity */
	class Controller : public Entity
	{
		private:

			Pawn* pawn;

		public:

			/** Initializes a Controller for the specified GameState */
			Controller(GameState* gs);
			virtual ~Controller();

			/** Gets the Pawn under this Controller's influence */
			Pawn* GetControlledPawn();
			/** Sets the Pawn under this Controller's influence */
			void SetControlledPawn(Pawn* pawn);

			/** Takes control over the specified Pawn */
			virtual void Possess(Pawn* pawn);
			/** Relinquishes control of the controlled Pawn, if there is any */
			virtual void Exorcise();

			/** Gets the ControlState of the controlled Pawn, if there is one */
			virtual ControlState* GetControlState();
	};

	/** Class representing a player's control over the player's Pawn */
	class PlayerController : public Controller
	{
		public:
			PlayerController(GameState* gs);
	};
}
