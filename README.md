# CarND-Path-Planning-Project Write-up

## Overall Algorithm
The general algorithm follows the solution proposed in the walkthrough at the end of the course.

On initialization, the server reads the map of the highway and stores the waypoints, which are placed in the middle of the double-yellow line in the center of the highway.

On each message from the client, the server parses the incoming data: 1) the ego car's localiation data, 2) the remainder of the previously-suggested path that has not yet been completed by the ego car, and 3) sensor fusion data describing the location of other vehciles.

Using sensor fusion data, the server makes predictions about the future positions of the other vehicles. These predictions, together with the ego car's localization data, are passed on to a Finite State Machine (described in the next section), which decides what the car should do next: either keep the current lane at the current speed, or keep the current lane and reduce the speed, or change lanes.

Based on this decision, a path plan is built as suggested in the walkthrough. First, a reference state is computed (either where the ego car currently is, or the end of the previous path). Next, three points are added to the path plan, spaced evenly 30m apart in the `s` Frenet coordinate. Now we have 4 points expressed in global Cartesian coordinates. The next steps are to convert these points to local Cartesian coordinates, then interpolate them using a spline.

At this point, we have a continuous path. The next step is to select a discrete set of 50 points on this continuous path such that their spacing ensures the ego car travels at maximum safe speed. Finally, these points are converted back to global coordinates, and returned by the algorithm.

## The Finite State Machine
The Finite State Machine implemented in `fsm.h`) consists of four states:
```
    LANE_KEEP
    SLOW_DOWN
    LANE_CHANGE_LEFT
    LANE_CHANGE_RIGHT
```
This state set is similar to the one described in the course, with the difference that the `PREPARE_LANE_CHANGE_LEFT` and `PREPARE_LANE_CHANGE_RIGHT` were merged into a single `SLOW_DOWN` state.

From `LANE_KEEP` and `SLOW_DOWN`, the system can transition into any of the 4 states. From `LANE_CHANGE_LEFT` and `LANE_CHANGE_RIGHT`, the system can only transition into `LANE_KEEP`. This restriction simplifies the mental model of what the car can do, without necessarily constraining its range of motion. It can still change two lanes virtually at once by following the sequence `LANE_CHANGE_LEFT -> LANE_KEEP -> LANE_CHANGE_LEFT`.

There are additional checks in place that take into account the current lane when deciding whether a particular next state is valid (for instance, when the current lane is `LEFT`, the state `LANE_CHANGE_LEFT` is not a valid next state). This is implemented in the `GetPossibleNextStates()` method.

### The Cost Functions
The transition function iterates over all the reachable next states and, for each, computes a cost. The state with minimum cost is selected as the next state.

The transition function makes use of two cost functions:
```
   CostOfChange()
   CostOfCollision()
```
`CostOfChange()` is straightforward: there's no cost in keeping the lane at constant speed, there is some cost in changing lanes (0.5) and there is a high cost in slowing down (1.0). These values make sure that the ego car doesn't tailgate a slow car, and prefers instead to change lanes.

`CostOfCollision()` takes into account the target lane of a state (e.g. the target of `LANE_CHANGE_LEFT` is the lane to the left of the current lane) and monitors the traffic on that lane. It incurs maximum cost when the projected position of the ego car is less than 30m behind another vehicle. Additionally, for the `LANE_CHANGE_LEFT` and `LANE_CHANGE_RIGHT`, it also incurs maximum cost when the projected position of the ego car is less than 10 m *in front* of another vehicle. This prevents the car from switching lanes when it's not safe to do so due to an incoming vehicle that might hit it from behind.

The two costs are summed up together, with weights of `0.05` and `0.95` respectively, thus prioritizing safety over efficiency.

# Result
The car is able to drive around the full track with no collisions, at maximum safe speed, without exceeding any of the jerk thresholds. [Here](https://www.dropbox.com/s/wxaqnyvn3ewak0j/project_video.mov?dl=0) is a video. It gets quite interesting between seconds 05:07 and 05:17, when it's trapped in a triangle: there's a car in the front, a black car on the left lane and a red car on the right lane. The ego car adapts its speed, until it finds a safe space to change lanes and get out of the triangle.

One drawback of my approach is that it's too greedy, and plans only one step ahead. This can easily lead to suboptimal solutions. For instance, say that the ego car is on the rightmost lane. In front of it, there's another car. In the middle lane, there's another car travelling at the same speed as the one on the rightmost lane. The ego car can't think two steps ahead (change left, then change left again). It will get stuck on the rightmost lane, tailgating the slow cars.

