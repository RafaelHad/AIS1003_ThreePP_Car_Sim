# AIS1003_ThreePP_Car_Sim


* Tank Simulator Project - by Rafael A.H.Haddal

* External Libraries used:
 - threepp (https://github.com/markaren/threepp): Used for 3D rendering and scene management.
 - Catch2 (https://github.com/catchorg/Catch2): Used for unit testing.

* Live coding session 07_10 was used as an initial starting point

* AI Tools used:
* Claude and Gemini was used as a research tool and helped implement some functions such as instanced generation, tank physics, dust clouds, and collision (+visualization).
* AI also helped set up unit tests and with complex maths.

* Assets used:
* - KayKit - Forest Nature Pack (https://kaylousberg.itch.io/kaykit-forest): Used for trees, grass, rocks, and clouds.

# How to Run

Press run 'live_07_10'

# Controls

W - Forward

A - Left

S - Reverse

D - Right

Space - Brake

O - Collider debug

R - Reset

C - Camera mode toggle

X - Up

Z - Down

Q - Pan Left

E - Pan Right

Tank control have two behaviours depending on speed

-pivot when stationary (each track moves in opposite directions)
-regenerative steering when in motion (one track slows down and transfers excess power to the other)

