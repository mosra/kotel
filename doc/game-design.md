Codename: Kotel
===============

Four Pillars
------------

1.  **Speed** -- get immersed into the game at high speeds and be rewarded for
    that
2.  **Physically based interaction** -- your vehicle is controlled only by its
    jets and brakes and you need to develop your unique driving technique to
    master the game
3.  **Non-linear gameplay** -- whether you like one game mode better than other
    or get stuck somewhere, you always have at least one other option to advance
    forward
4.  **Vehicle modding** -- make your vehicle the fastest or the most flexible to
    suit your unique driving technique

Basic game principle
--------------------

High speed vehicle in long curved tube. The vehicle is basically body with two
arms touching the pipe (imagine clock with hands on 4 and 8). The central part
has one jet responsible for moving forward, each arm have one jet responsible
for tilting to left and right and also brakes for fast decceleration. The goal
is to develop and maintain driving technique allowing to go as fast as possible
without bumping into walls and obstacles.

The player has an **experience level**, which is upgraded during the gameplay.
Experience level is used to unlock better vehicles and next levels (each vehicle
and level has minimal required experience to be unlocked).

It is possible to do these achievements:

*   Achieving some critical speed and maintaining it for some time
*   Crazy trick or collision avoidance (e.g. spiral motion)
*   Successfully completing a level for the first time
*   Creating new time record on already completed level

Each achievement is rewarded either by increasing player experience or adding
vehicle upgrade possibility (which leads to higher speeds or more flexibility,
which then leads to increased player experience).

World features
--------------

Tube with:

-   **Curves** -- centrifugal force influences the vehicle
-   **Sharp turns** -- some technique needed to get past them without crashing
-   **T-junctions** -- advanced techniques needed to turn, easy to go forward
-   **Diameter changes** -- more different techniques can be used to pass them,
    some can have a special section suited for path forward

Also see [Obstacles](#obstacles).

Interaction between vehicle and the world
-----------------------------------------

The vehicle is essentialy driven by these forces:

-   **Gravity** -- it is not possible to drive upside down if the other forces
    can't compensate for it, leading to crash
-   **Intertia** -- i.e., it's harder to accelerate/decelerate than maintain
    constant speed
-   **Friction** -- rougher surfaces will decelerate the vehicle, having inequal
    force pushing on both arms will result in speed reduction or even crash in
    extreme cases
-   **Jet engines** -- the central makes forward motion and the side ones tilt
    it to left/right, controlled by user, efficiency depends on vehicle
    parameters
-   **Brakes** -- on the arms, also controlled by user, their efficiency depends
    on vehicle parameters and friction between arms and tube
-   **Centrifugal force in curved tubes** -- the vehicle will be pushed to one
    side, causing inequal force on its arms or leading to crash

No aerodynamics or drag forces are taken into account, the tracks are purely
horizontal to avoid complex physics interactions.

If the vehicle _touches_ anything with its central part (i.e. falling on it due
to not obeying gravity rules), bumps with higher than _harmless_ speed into
something (i.e. an obstacle or wall) or has _excessive arm load_ it crashes and
the game is over. The goal is to have both arms adherent to tube surface and
have equal force pushing on both of them.

Vehicle definition
------------------

The vehicle is defined by:

-   **Mass**, separately for central part and arms
-   **Power** of central jet engine and arm jet engines
-   **Arm friction/adhesion**, influencing both how fast it can accelerate and
    decelerate, what force is needed to maintain constant speed and what force
    the arms can hold until they unstick from the surface

Each vehicle has slightly different properties, suitable for different game
modes. Weight distribution between central part and arms considerably affects
vehicle behavior.

Controlling the vehicle
-----------------------

By controlling the central jet and enabling jet or brakes on either left or
right arm the vehicle can be pushed forward or backward and tilted to the left
or to the right. Tilting can be used to avoid collisions or to react to
centrifugal and gravity forces.

When tilted far enough with sufficient speed the vehicle can be brought into
spiral motion, which can be then used for some non-trivial tricks:

*   High forward speed can be transformed into high rotation speed with almost
    no forward motion, suitable when normal deceleration is not an option (oily
    surface...)
*   Different vehicle orientation when spinning can be used to turn into
    T-junctions or do very sharp turns without crashing or damaging the vehicle

Game modes
----------

The game is not linear, each game mode has its own unlocking path and the
player can always choose from at least two new levels from two different modes.

1.  **Technique** -- complete level with given amount of time and avoid
    collisions, basically a well-balanced mix of the following two modes with
    more power-ups
2.  **Time round** -- collect checkpoints as fast as possible, timeout to next
    checkpoint, penalties for not collecting some checkpoints, tricky
    checkpoints on unusual places, not so many obstacles
3.  **Slalom** -- successfully complete the level without crashing, no time
    limit (but some areas can't be just speedlessly "walked through") and
    increased count of obstacles
4.  **Survival mode** -- increasing speed and difficulty, getting as far as
    possible. These levels essentialy can't be finished (it always gets to the
    point where the player crashes), so they aren't part of any unlocking path,
    but they contribute to increasing player experience. They are unlocked as a
    bonus for outstanding achievements.

Each following level is unlocked by both completing the previous one in given
path and having large enough *experience level*. Getting far enough in one path
might as well unlock earlier levels on different paths to avoid player getting
stuck at one unbreakable level for too long. For example completing level 6 in
*Slalom* might unlock level 3 with in *Time round*.

Obstacles
---------

Obstacles either reduce player speed or lead to unrecoverable crashes and game
over:

-   **Dirt**, **rust** or **tar stain** -- temporarily reduces player speed
-   **Gate**, **wire**, ... -- closing some sector of the circle, might be
    animated, unrecoverable crash
-   **Screw**, **bolt**, **weld**, ... -- depending on size and area which was
    hit it might either reduce speed or lead to crash

Power-ups
---------

Power-ups can temporarily improve some aspects of either level or vehicle, but
they don't add anything to player experience. When used inappropriately, they
act as *power-downs* (ahem).

Artificial power-ups are "glowy" things appearing randomly in the level, might
react co current player state (compensating for inappropriate speed etc.):

+   **Speed boost** -- adds immediate speed boost (however it might lead to
    immediate crash)
+   **Vehicle magic** -- temporarily changes friction, modifies jet power etc
    (in both positive and negative ways)
+   **Obstacle removal** -- burns out oil stains, explodes gates (but might
    damage the vehicle as well)

Real power-ups appear on predefined locations and are integral part of the
level:

+   **Oil stain** -- player have better acceleration on it (but worse
    deceleration)
+   **Button** -- **green** one stops gate from closing or starts opening closed
    gate, **red** one starts closing it
+   **Exit sign** -- hints the best way in T-junctions and crossroads

Vehicle modding
---------------

The player is able to modify the vehicle to suit the needs of particular driving
technique.

-   More power with larger jets or better weight distribution with smaller ones
-   Better brake system or lower friction for higher speeds

Features to ~~remove~~ postpone
-------------------------------

So I don't go haywire from doing all that at once :-)

-   **Brakes** -- make them available only in later vehicle models, the first
    models will have such massive friction that braking can be done just by
    stopping the central jet
-   **Other game modes** and **power-ups** -- add them when the core mechanic is
    really working and all issues are ironed out, the other modes are in fact
    just specialized variants of the first one
