#include "StdAfx.h"
#include "Control.h"

namespace CibraryEngine
{
	/*
	 * ControlState methods
	 */
	ControlState::ControlState() { }

	ControlState::~ControlState() { }



	/*
	 Pawn methods
	 */
	Pawn::Pawn(GameState* gs) : Entity(gs), controller(NULL), control_state(NULL) { }
	Pawn::~Pawn() { }




	/*
	 * Controller methods
	 */
	Controller::Controller(GameState* gs) : Entity(gs), pawn(NULL) { }
	Controller::~Controller() { }

	Pawn* Controller::GetControlledPawn() { return pawn; }

	void Controller::SetControlledPawn(Pawn* pawn_)
	{
		if(pawn != pawn_)
			pawn = pawn_;
	}

	void Controller::Possess(Pawn* pawn_)
	{
		Exorcise();
		pawn = pawn_;
		pawn->controller = this;
	}

	void Controller::Exorcise()
	{
		if (pawn != NULL)
			pawn->controller = NULL;
		pawn = NULL;
	}

	ControlState* Controller::GetControlState() { return pawn != NULL ? pawn->control_state : NULL; }




	/*
	 * PlayerController methods
	 */
	PlayerController::PlayerController(GameState* gs) : Controller(gs) { }
}
