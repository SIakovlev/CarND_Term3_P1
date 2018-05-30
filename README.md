[image1]: ./pics/simple_overtaking1.png
[image2]: ./pics/simple_overtaking2.png
[image3]: ./pics/simple_overtaking3.png
[image4]: ./pics/overtaking_surrounded1.png
[image5]: ./pics/overtaking_surrounded2.png
[image6]: ./pics/overtaking_surrounded3.png

# CarND_Term3: Path-planning project

## Compilation and building instructions

* Clone this repository
* Make a build directory: `mkdir build && cd build`
* Compile the project with `cmake .. && make`
* Run it: `./path_planning`

## Project criterias

* The car is able to drive at least 4.32 miles without incident..
The screenshots below demonstrate that it was driving without collision for more than 4.32 miles.

Before, during and after overtaking:
<p float="left">
  <img src="/pics/simple_overtaking1.png" width="270" />
  <img src="/pics/simple_overtaking2.png" width="270" /> 
  <img src="/pics/simple_overtaking3.png" width="270" />
</p>

Another case is when the ego car is surrounded by other vehicles. Then it keeps distance with the car ahead and changes lanes only when it is safe (i.e. there are no vehicle ahead and behind):
<p float="left">
  <img src="/pics/overtaking_surrounded1.png" width="270" />
  <img src="/pics/overtaking_surrounded2.png" width="270" /> 
  <img src="/pics/overtaking_surrounded3.png" width="270" />
</p>

* The car drives according to the speed limit.

The simplest P controller was implemented. It provides satisfying performance allowing quickly breaking if there is a car in front of the ego vehicle, and accelerate up to a speed limit when the road is empty.

* Max Acceleration and Jerk are not Exceeded.

To ensure that max acceleration is not exceeded, the aforementioned P controller output is bounded from above. For implementation details see lines ` `.

Regarding jerk minimisation trajectories, the same approach as in the video lessons was implemented (see lines ` `). 

* Car does not have collisions.

There is a fairly simple state machine. The ego car is always monitoring vehicles at each lane in the range of distances 20 m ahead and 10 meters behind.



