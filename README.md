[image1]: ./pics/simple_overtaking1.png
[image2]: ./pics/simple_overtaking2.png
[image3]: ./pics/simple_overtaking3.png
[image4]: ./pics/overtaking_surrounded1.png
[image5]: ./pics/overtaking_surrounded2.png
[image6]: ./pics/overtaking_surrounded3.png
[image7]: ./pics/map_data.png
[image8]: ./pics/x_s_and_y_s.png

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

Another case is when the ego car is surrounded by other vehicles. It keeps distance with a car ahead and changes lanes only when it is safe (i.e. there are no vehicle ahead and behind):
<p float="left">
  <img src="/pics/overtaking_surrounded1.png" width="270" />
  <img src="/pics/overtaking_surrounded2.png" width="270" /> 
  <img src="/pics/overtaking_surrounded3.png" width="270" />
</p>

**The car drives according to the speed limit**.

The simplest P controller was implemented (see lines `204-214` in `planner.cpp`). It provides satisfying performance allowing quickly breaking if there is a car in front of the ego vehicle, and accelerate up to a speed limit when the road is empty.

Future improvement: 
- MPC controller (although this would require more accurate waypoits data)

**Max Acceleration and Jerk are not Exceeded**.

To ensure that max acceleration is not exceeded, the aforementioned P controller output is bounded from above. For implementation details see lines `204-214` in `planner.cpp`.

Regarding jerk minimisation trajectories, the same approach as in the video lessons was implemented. The optimisation problem is formulated as the minimisation of the third derivative of coordinate and has a well-known solution in the form of [quintic polynomials](https://en.wikipedia.org/wiki/Quintic_function). Given initial conditions for coordinate, speed and velocity in (s, d) coordinates, coefficients of this polynomial can be found by solving a system of linear equaions. For implementation details see `Planner::jmt` and `Planner::jmt_coefficients` methods in `planner.cpp`. After that trajectory points can be generated and fitted using c++ [spline library](http://kluge.in-chemnitz.de/opensource/spline/). This gives smooth trajectories even when car changes lanes consequently.

**Car does not have collisions**.

In current realisation, the collision avoidance algorithm is based on empirical ideas. The ego car is always monitoring vehicles at each lane in the range of distances 20 m ahead and 10 meters behind - if there are cars in the proximity of the ego vehicle, then it is dangerous to turn. If there is a car in front of the ego vehicle it keeps distance maintaining the same speed. It also measures average lane speeds based how many cars it can detect on each lane and calculates the cost of changing lane. 

Future improvement: 
- collision avoidance algorithm based on Separating Axis Theorem (SAT) 
- cost functions that can give low cost, when there is an empty lane. It will also require more complexity in the state machine so that the car can change lanes twice (or many times in a row) if needed.
- cost functions that assess the lane based on closest car's predicted behaviour.

**The car stays in its lane, except for the time between changing lanes**.
The ego car reliably stays on the lane, however a better speed/position controller (see above) could have improved performance.

**The car is able to change lanes**
See screenshots where this is shown.

## Future work
Apart from previously mentioned improvements I'm planning to focus on a few more:

* utilise interpolated waypoints in order to get a better accuracy and more effective trajectory generation. For example, `x(s)` and `y(s)` can be fitted using splines:
![][image8]
this would result on much more accurate conversion from (x,y) to (s,d) coordinates.

* Behaviour prediction module for prediciton of sudden lane changes for other cars.
* Implement a proper trajectory search, by generating multiple trajectories and choosing the most optimal one.
* Minimise cost function consisting of jerk + some additional term, i.e. elapsed time (as in the paper [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame](http://video.udacity-data.com.s3.amazonaws.com/topher/2017/July/595fd482_werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame/werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame.pdf))




