#include "StdAfx.h"
#include "Control.h"

namespace CibraryEngine
{
	/*
	 * ControlState methods
	 */
	ControlState::ControlState() : float_controls(), bool_controls() { }

	ControlState::~ControlState()
	{
		for(map<FloatControlChannel*, Control<float>*, ControlChannelComp>::iterator iter = float_controls.begin(); iter != float_controls.end(); ++iter)
			delete iter->second;
		for(map<BoolControlChannel*, Control<bool>*, ControlChannelComp>::iterator iter = bool_controls.begin(); iter != bool_controls.end(); ++iter)
			delete iter->second;
	}

	Control<float>& ControlState::operator [](FloatControlChannel& channel)
	{
		map<FloatControlChannel*, Control<float>*, ControlChannelComp>::iterator iter = float_controls.find(&channel);
		return *iter->second;
	}

	Control<bool>& ControlState::operator [](BoolControlChannel& channel)
	{
		map<BoolControlChannel*, Control<bool>*, ControlChannelComp>::iterator iter = bool_controls.find(&channel);
		return *iter->second;
	}

	void ControlState::AddFloatControl(FloatControlChannel& channel) { float_controls.insert(pair<FloatControlChannel*, Control<float>* >(&channel, new Control<float>(&channel))); }

	void ControlState::AddBoolControl(BoolControlChannel& channel) { bool_controls.insert(pair<BoolControlChannel*, Control<bool>* >(&channel, new Control<bool>(&channel))); }




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
