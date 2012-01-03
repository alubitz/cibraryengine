#include "StdAfx.h"
#include "Control.h"

#include "GameState.h"

namespace CibraryEngine
{
	/*
	 * ControlState methods
	 */
	ControlState::ControlState() { }
	ControlState::~ControlState() { }

	// TODO: implement these
	bool ControlState::GetBoolControl(string control_name) { return false; }
	float ControlState::GetFloatControl(string control_name) { return 0.0f; }
	void ControlState::SetBoolControl(string control_name, bool value) { }
	void ControlState::SetFloatControl(string control_name, float value) { }



	/*
	 Pawn methods
	 */
	Pawn::Pawn(GameState* gs) : Entity(gs), controller(NULL), control_state(NULL) { }
	Pawn::~Pawn() { }




	/*
	 * Controller methods
	 */
	Controller::Controller(GameState* gs) : Entity(gs), pawn(NULL), ctrl_update_interval(0.1f) { next_ctrl_update = last_ctrl_update = gs->total_game_time; }
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

	void Controller::Update(TimingInfo time)
	{
		float now = time.total;

		if(now > next_ctrl_update)			// using > in case time.elapsed is 0 somehow
		{
			UpdateController(TimingInfo(now - last_ctrl_update, now));

			last_ctrl_update = now;
			next_ctrl_update = now + ctrl_update_interval;
		}
	}




	/*
	 * PlayerController methods
	 */
	PlayerController::PlayerController(GameState* gs) : Controller(gs) { ctrl_update_interval = 0.0f; }

	void PlayerController::UpdateController(TimingInfo time) { }
}
