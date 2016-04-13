# Originally Proposed Weekly Schedule #

  * Week 1: Calling Lua Functions from C++
  * Week 2: Lua Interfacing with C++
  * Week 3: Pathfinding
  * Week 4: Goal-Driven Agent Behavior
  * Week 5: Fuzzy Logic
  * Week 6: Scripted Mission Events
  * Week 7: Inverse Kinematics
  * Weeks 8, 9: Network Transmission & Synchronization
  * Weeks 10, 11: Preparing Final Game

# Actual Progress #

## Calling Lua Functions from C++ / Lua Interfacing with C++ ##

There's scripting all over the place, now.

There are a handful of hard-coded scripts (located in Files/Scripts/) that are run at certain points in the execution of the program:

  * globals.lua, run when the program starts up
  * load.lua, run when you start a game (during the loading screen)
  * game\_start.lua, run prior to the first frame of actual gameplay
  * update.lua, called every frame

There are a number of functions exposed to Lua. There is a vector class (with most of the normal vector arithmetic operators defined), a println function, etc. It is possible to spawn the player, or bugs, from Lua.

It is also possible to set a Lua function as a callback for certain Dood events: per-frame update and ai callbacks, and a callback triggered when a Dood dies.

## Pathfinding ##

The PathSearch class in CibraryEngine/Pathfinding.cpp contains an implementation of the A`*` search algorithm; the class has methods to solve an entire problem, or to perform some number of iterations on the problem. This class is accessible to scripting.

Underneath the pathfinding system is the NavGraph system, which is also exposed to scripting.

For more info on pathfinding, [see here](Pathfinding.md)

## Goal-Driven Agent Behavior ##

Goal-driven agent behavior (loosely based on the implementation in the book _Programming Game AI by Example_ by Mat Buckland) is in Files/Scripts/goals.lua

There is room for improvement, but it works.

## Fuzzy Logic ##

I didn't have time for this due to other projects.

## Scripted Mission Events ##

Although it isn't much like what I had originally envisioned, I did create the gs.showChapterText function, which is used to display "Wave 1", "Wave 2", etc.

Having mission-specific scripts should probably be doable without too much difficulty.

## Inverse Kinematics (IK) ##

I never managed to get proper IK working, but I was able to create a framework for it which will greatly facilitate the eventual implementation of inverse kinematics.

Effectively all that remains is the implementation of a single function, the `IKSolver::IKObject::ComputeNextState` function in CibraryEngine/IKSolver.cpp, which has access to all of the data it will probably need.

## Network Transmission & Synchronization ##

I have ported some of my C# networking code to C++ using boost::asio, and after bugfixing the multi-threading and a number of other problems, I created a simple test program within the framework, `Network::DoTestProgram()` in CibraryEngine/Network.cpp, and it seems to work.

As for actually making the game multiplayer, I haven't started that yet, but I imagine that a lot of the concepts I used in Multiplayer Laser Battle (MPLB) will be applicable for this game as well. There is one thing I'm not so sure about, which is how to synchronize the game physics, which was significantly more complicated than the physics of MPLB... but I'll deal with that when I get to it.