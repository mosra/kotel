Codename: Kotel
===============

This is work-in-progress tunnel racing game. Nothing cut in stone yet, but some
things are already specified -- go read [game design document](doc/game-design.md)
now.

The game runs on [Magnum](https://github.com/mosra/magnum) OpenGL engine.

Demos and prototypes
--------------------

Various demos, physics and gameplay prototypes are available in `prototypes/`
subdirectory and also online running in Google Chrome Native Client:

*   **Forces2D** -- http://mosra.github.io/kotel/forces-2d.html -- 2D physics
    interaction with gravity, engine forces and collision detection. Use
    **left/right arrow** to control the engines and **R key** to reset the
    vehicle.

You need Google Chrome 22 or higher with working Native Client and OpenGL.
Native Client is disabled by default and only apps from Chrome Web Store are
allowed to run. This can be solved either by enabling Native Client in
[chrome://flags](chrome://flags) or running chrome from terminal with
`--enable-nacl` option.

You can check that OpenGL is working on this [native client example](http://gonativeclient.appspot.com/dev/demos/sdk_examples/fullscreen_tumbler/fullscreen_tumbler.html).
It should display a cube which can be rotated using mouse. Some GPUs are
blacklisted, you can try to bypass it with `--disable-gpu-blacklist` option.

LICENSE
=======

Kotel is licensed under MIT/Expat license, see [COPYING](COPYING) file for
details.
