## Graphics Code Changes ##

The main big things I plan to do with the graphics code are as follows:

  * ~~Shadows~~ Done as of 19 Dec 2011

  * Vertex buffer changes
    * Indices (useful for depth-sorting transparent primitives)
    * Proper support for points and lines, not just triangles
    * Instancing

  * Material system refactoring
    * New Material class with the following members:
      * Blend style
      * Shader program
      * Cast shadows (y/n)?
    * Revise the material files system