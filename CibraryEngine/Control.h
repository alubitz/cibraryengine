#pragma once

#include "StdAfx.h"
#include "Entity.h"

namespace CibraryEngine
{
	using namespace std;
	using boost::unordered_map;

	class Controller;

	/** The collection of controls for a Pawn */
	class ControlState
	{
		private:

			unordered_map<string, float> control_values;

		public:

			/** Initializes a control state */
			ControlState();
			~ControlState();

			bool GetBoolControl(const string& control_name);
			float GetFloatControl(const string& control_name);
			void SetBoolControl(const string& control_name, bool value);
			void SetFloatControl(const string& control_name, float value);
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

			float last_ctrl_update, next_ctrl_update;
			float ctrl_update_interval;

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

			/** Causes UpdateController to be caused when sufficient time has elapsed */
			void Update(TimingInfo time);
			/** Abstract function to let a controller update the control state */
			virtual void UpdateController(TimingInfo time) = 0;
	};

	/** Class which executes a lua script to determine the ControlState for a Pawn */
	class ScriptedController : public Controller
	{
		public:

			/** The filename of the script to be executed */
			string script;

			/** Creates a Controller which will execute the script with the given filename */
			ScriptedController(GameState* gs, const string& script);

			void UpdateController(TimingInfo time);
	};
}
