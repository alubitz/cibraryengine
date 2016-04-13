# List of Planned Features... #

## Gameplay... ##

  * Player control customization (low priority)
  * Mission-specific scripts
  * For the humans...
    * AI teammates
    * Grenades
    * Health regeneration
  * For the bugs...
    * Artillery bug
    * Bug holes
  * Damage model changes
    * Vital parts
    * Region-specific damage resistance
    * Destructible parts (armor plates, amputations, etc.)

## Multiplayer... ##

  * Change menus for multiplayer
  * Networking stuff
  * Esc menu

## Physics... ##

  * Fix ragdoll slowdown
  * Dood physics redux
    * Different collision shapes for walking / weapon hit
    * Major rewrite?
    * Jump should apply an opposite force to the thing you are standing on (low priority)

## Graphics... ##

  * Geometry shaders and transform feedback
  * Vertex buffer stuff
    * Indices (useful for depth-sorting transparent primitives)
    * Proper support for points and lines, not just triangles
    * Instancing
  * Material system refactoring
  * Decals

## AI... ##

  * Fix spinning when idle
  * Fight-or-flight behavior using fuzzy logic
  * Path smoothing
  * Address pathfinding performance issues
    * Groupthink?
    * Hierarchical system?

## Backend... ##

  * Detect and eliminate memory leaks
  * Make stuff less hard-coded
  * Content system refactoring
    * Maybe get rid of ContentMan singleton altogether?
    * Implement proper usage tracking and unloading
      * Maybe force all content types to implement an interface
    * Stop using Texture2DLoader to load content outside of /Textures
    * File system
  * Graphics compatibility stuff

## Miscellaneous... ##

  * Inverse kinematics
  * Model conversion util
  * Scenery
    * Shrubs
    * Trees
  * More sound effects

# Priority Goals #

## Model Conversion Utility ##
I need a utility to import and export to/from my custom .zzz model format. I already can import the simple .obj model format, but this is insufficient for skinned models or animations, so I'm going to make an importer for UnrealEngine's .psk and .psa files.

## Multiplayer ##
Multiplayer is a big task, and I don't entirely expect I'll have something working by the end of this term.

## Refactoring / Making things less hard-coded ##
In order to do certain things like having more than one type of enemy, a lot of hard-coded data really ought to be moved to external files, or scripted. This is especially true for properties of the Dood and Weapon classes.