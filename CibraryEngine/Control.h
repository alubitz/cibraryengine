#pragma once

#include "StdAfx.h"
#include "Entity.h"

namespace CibraryEngine
{
	using namespace std;

	class Controller;

	// A control "channel" is a prototype specifying the name and usage of a type of control
	/** Class representing a prototype name and usage of a type of control */
	class ControlChannel
	{
		public:
			/** The name of this control channel */
			string control_name;

			/** Initializes a ControlChannel with the specified name */
			ControlChannel(string control_name) : control_name(control_name) { }

			virtual ~ControlChannel() { }
	};

	// struct for comparing control channels
	struct ControlChannelComp { bool operator ()(const ControlChannel* L , const ControlChannel* R) { return L->control_name.compare(R->control_name) < 0; } };

	// I is the [I]nternal storage type
	// E is the [E]xternal type you use it as
	/** A control channel which can be assigned to/from using the specified type E, optionally using a different internal format I */
	template <typename E, typename I = E> class TypedControlChannel : public ControlChannel
	{
		public:

			/** Initializes a TypedControlChannel with the specified name */
			TypedControlChannel(string control_name) : ControlChannel(control_name) { }
			virtual ~TypedControlChannel() { }

			/** Abstract function to set the value of the control */
			virtual void Set(I& target, E value) = 0;
			/** Abstract function to get the current value of the control */
			virtual E Get(I& source) = 0;

			/** Abstract function to get the default value of the control, for Control initialization; returns internal type, not external! */
			virtual I Default() = 0;
	};

	/** A control with boolean values */
	class BoolControlChannel : public TypedControlChannel<bool>
	{
		private:
			bool default_value;

		public:

			/** Initializes a BoolControlChannel with the given name and default value */
			BoolControlChannel(string control_name, bool default_value) : TypedControlChannel<bool>(control_name), default_value(default_value) { }

			void Set(bool& target, bool value) { target = value; }
			bool Get(bool& source) { return source; }

			bool Default() { return default_value; }
	};

	/** A control channel with float values clamped to a range */
	class FloatControlChannel : public TypedControlChannel<float>
	{
		private:
			float default_value;
			float minimum;
			float maximum;

		public:

			/** Initializes a FloatControlChannel with the given name, default value, and minimum and maximum values */
			FloatControlChannel(string control_name, float default_value, float minimum, float maximum) : TypedControlChannel<float>(control_name), default_value(default_value), minimum(minimum), maximum(maximum) { }

			void Set(float& target, float value) { target = max(minimum, min(maximum, value)); }
			float Get(float& source) { return source; }

			float Default() { return default_value; }

			/** Returns the minimum value for this control */
			float Minimum() { return minimum; }
			/** Returns the maximum value for this control */
			float Maximum() { return maximum; }
	};

	/** A BoolControlChannel used to control actions */
	class ActionControlChannel : public BoolControlChannel
	{
		public:
			virtual void Execute(TimingInfo time);
	};

	/** Class representing an instance of a control */
	template <typename E, typename I = E> class Control
	{
		public:
			/** The control channel this is an instance of */
			TypedControlChannel<E, I>* channel;
			/** The internal value for this control... uh... what? */
			I internal_value;

			/** Initializes a control from its prototype */
			Control(TypedControlChannel<E, I>* channel) : channel(channel), internal_value(channel->Default()) { }

			/** Allows the Control to be treated as its external type by casting it as that type */
			operator E() { return channel->Get(internal_value); }
			/** Allows the Control to be assigned to with a value of its external type */
			void operator =(E e) { channel->Set(internal_value, e); }
	};

	/** The collection of controls for a Pawn */
	class ControlState
	{
		protected:

			/** Controls whose values are clamped floating-point numbers */
			map<FloatControlChannel*, Control<float>*, ControlChannelComp> float_controls;
			/** Controls whose values are booleans */
			map<BoolControlChannel*, Control<bool>*, ControlChannelComp> bool_controls;

		public:

			/** Initializes a control state */
			ControlState();
			~ControlState();

			/** Get or set the value of a clamped floating-point control */
			Control<float>& operator [](FloatControlChannel& channel);
			/** Get or set the value of a boolean control */
			Control<bool>& operator [](BoolControlChannel& channel);

			/** Add a clamped floating-point control */
			void AddFloatControl(FloatControlChannel& channel);
			/** Add a boolean control */
			void AddBoolControl(BoolControlChannel& channel);
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
