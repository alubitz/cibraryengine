## Physics Changes ##

There are a few things I need or want to do with the physics code.

One of the most significant of these changes is **fixing the ragdoll slowdown**, i.e. the lag that occurs when there are several active ragdolls in the scene at once.

Something else would be revising the **Dood physics**. The engine shouldn't use the low-detail collision shape it uses for the movement of the controller for detecting whether shots hit or not. Some revision of this code might be in order... however, since what I have now _works_, it's not a particularly high priority.