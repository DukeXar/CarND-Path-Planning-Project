# Path Planning Project Model Documentation

The path planning application consists of several modules:

- Map
- Trajectory
- Planner
- Decider
- World

The Planner consumes sensors fusion information from the simulator, it then uses World to model the position of other vehicles in the future using the sensors fusion information. The current vehicle position, world, and previous trajectory is fed into the Decider to choose the best trajectory to follow, and the results are then fed back into the simulator. The Decider chooses what to do: to keep speed, to follow other vehicle, or to change lanes, based on the information from the World, current state, and possible trajectories. It uses the Trajectory module to generate best trajectories for required maneuver, and chooses the one that does not violate speed, acceleration, jerk limits and does not collide with other vehicles.


## The Map module

The Map source code is located in the map.cpp and map.h files. It provides the Map class that encapsulates the map of the track, and can convert coordinates in Frenet frame into Cartesian coordinates using `FromFrenet` method.

The waypoints are loaded from the csv file into the map using `ReadMap`. After all waypoints were read, it also adds one additional waypoint that duplicates the first waypoint of the track, but has different *S* coordinate. This allows to smoothly convert Frenet coordinates into Cartesian at the end of the track. The `Freeze` method is used after the map was read and all waypoints were added to build the two sets of splines that allow to smoothly do the conversion of (s, d) pairs into (x, y).

## The Trajectory module

The approach for trajectories generation was taken from the [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame](https://d17h27t6h515a5.cloudfront.net/topher/2017/July/595fd482_werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame/werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame.pdf).

The Trajectory source code is located in the trajectory.h and trajectory.cpp files. It defines `JerkMinimizingTrajectory` function that returns a quintic polynomial, satisfying jerk minimizing condition, start, end states, and time. For the *keeping speed* mode, as it does not have a boundary condition on the position, the `JerkMinimizingTrajectory4` is used to generate quartic polynomial for speed.

The `FindBestTrajectories` returns the best trajectory given start state, moving target object, and cost function. It scans over multiple timestamps to simulate the state of the target in future, for every such state it generates more combinations of it, and then uses the `JerkMinimizingTrajectory` to generates a trajectory for each. Every generated trajectory is evaluated using the cost function, and then the trajectory that has the minimum cost is returned. The result is two trajectories in Frenet coordinates - one for longitudial, and another for laterial movement. It also returns the time it takes for the trajectory to complete, and its cost.

The `Target` interface that is passed into the function allows to use the same trajectory search function for the targets that, for instance, are moving at constant speed, or are accelerating with constant acceleration.

## The Planner class

The Planner class is implemented in the `planner.cpp` and `planner.h` files. The transport layer in `main.cpp` would call the `Update` function, providing all the information from the simulator - sensors fusion, unprocessed trajectory, position of the car.

After the trajectory was planned, the planner would store it, inclusing both Frenet coordinates, and Cartesian coordinates. It will then use the receivied unprocessed trajectory to find out how many points the simulator has processed. Because the simulator is a perfect model, there is no need to use the car position information from the simulator, besides the very first update. The planner needs to know the future trajectory so that would it decide to replan, it can do it smoothly. It also uses the stored trajectories to predict where the car would be in the future, so that it can account for its own latency. In my experiments, the usual latency of the planner when changing lanes, is about 0.02 seconds, rounding up to 0.1 seconds, the planner uses this value (`kAlgorithmLatencySeconds`) to predict where the car would be that time, and to start planning new trajectories from there. The planner replans the trajectories every `kReplanPeriodSeconds` (0.1) seconds. Doing it so frequently allows it to well react to changes in the surrounding environment, at the expense of a bit more comptutations.

## The Decider class

The class implements the behavioral model of the car.

The Decider class is implemented in the `planner.cpp` and `planner.h` files. The entry point is `ChooseBestTrajectory` that returns an array of trajectories, that should be concatenated and processed one after another. During update, the planner discovers and finds out the index of the trajectory it is processing now. It basically calls another `ChooseBestTrajectory` with new start state, time offset, new mode and parameters.

Depending on the mode, it either handles it with `HandleChangingLaneState` (lines 720-730), or chooses the optimal trajectories for lane keeping mode (732-812).

The total length of the predicted trajectories should be at least `kMinTrajectoriesTimeSeconds`, to accomodate possible latencies in the algorithm. Because of the replanning, one of the modes, lane changing specifically, might want to generate a very short trajectory (that happens at the end of the maneuver), that means that one trajectory could be not enough.

### Keeping Lane Mode

In this mode, the car just drives freely in its lane, trying to achieve the target maximum speed, while being safe.

It monitors the distance to the vehicle in front of it, and speed of the nearest vehicles in each lane, and can generate a few trajectories:

1. Keeping speed trajectory (`BuildKeepSpeedTrajectory`)
1. Keeping distance trajectory (`BuildKeepDistanceTrajectory`)
1. Changing lanes trajectory (`BuildChangingLaneTrajectory`)

To choose what is the best option to do, the following approach is implemented:

1. Model the position of other vehicles in current time offset.
1. If current lane has the highest speed, stay in it.
1. Add keep speed trajectory into consideration.
1. If there is a vehicle in front, add keep distance trajectory into consideration list.
1. If current lane does not have the highest speed, build switch lane trajectory.
1. If the lane switch trajectory has feasible cost (< 200), switch to lane switch mode and execute the trajectory.
1. Otherwise, from the list of trajectories to consider, choose the one with lowest score and execute it, staying in keeping lane mode.

The code to generate the keeping speed trajectory is implemented in `BuildKeepSpeedTrajectory` method. As it was mentioned before, it uses quartic polynomial to fit it. Amont possible candiates, it would choose the one that has average speed closer to target speed, is close to lateral position on the road, heavily penalizes distance to other vehicles and being outside of the lane, that it would travel on constant speed. It also includes acceleration and speed limits into the cost. It also would choose the trajectory that is closer to the center of the lane.

The code to generate the following a vehicle mode is implemented in `BuildKeepDistanceTrajectory` method. It models the target vehicle as a constant speed target, which works well in most of the cases. Same way as in keeping speed mode, it penalizes being outside of the lane, and hitting other vehicles, tries to be closer to the center of the lane, not exceed the acceleration and speed limits. In this trajectory, it is also important to be around desired S and D states.

### Switching Lanes Mode

The code for lanes switching trajectory is implemented in `BuildChangingLaneTrajectory` method. It tries more options that other two trajectories, because it should be a more precise operation. It also generates shorter trajectories (5 seconds at most). For the cost, it tries to stay in between the boundaries of two lanes, have good distance to other cars, prefer shorter trajectories.

To switch the lanes, it requires to know the source state, target lane, target speed. If the target lane is occupied, the target speed would be the speed of the next car in the lane. When it is not occupied, based on experiments, seems better to keep the current speed and switch.

The switch is considered complete when target D position is +- 0.2M from the lane center.

### Cost Functions

Here I will describe some details about trajectories cost functions and why they were chosen this way.

#### ClosenessCost

This is logistic sigmoid function that is used to measure closeness of the perturbed target state to the one generated by the trajectory function.

#### CartesianAccelerationLimitCost

This function converts S and D trajectories to the Cartesian coordinate system, and calculates average velocity and acceleration, trying to replicate the same averaging method as simulator. When speed or acceleration is below 90% of the limit, the cost is 0. For the last 10%, the cost increases as power of 5 function. Whenever speed or acceleration is higher than limit, the function returns huge value of 100 (vs 1 for other cost functions), to try to avoid such trajectory at all costs.

The average speed and acceleration is calculated using `GetMaxCartesianAccelerationAndSpeed`, which uses map to convert Frenet points into Cartesian. Values are averaged every 10 points (0.2 seconds), and then reset (i.e. it is not a sliding window). For the acceleration, both tangential and normal values are calculated.

#### ExceedsSafeDistance

This cost function takes S and D trajectories, world model, map and lanes configuration. The lanes configuration indicates in which lanes it should check the distance (i.e. it doesn't care about cars in lane 0, when it is in the lane 2, also it checks nearby lane only on lane change maneuver, otherwise it checks just the current lane). The configuration also indicates whether distance to the car behind should be checked or not, it is important to do that when switching lanes, so that the car behind wouild not run into our car. But when keeping the distance, it was leading to false positives, so that the model checks only the car in front.

When checking the car behind, the speed of that car is used to determine the safest distance (`GetMinDistanceToKeep`). When checking the front car, our speed is used. The safe distance is calculated based on time it would take to reduce the speed from current to zero given maximum deceleration. Because the car behind is controlled by the simulator and does not complain about acceleration and jerk, I tried to set the minimum distance for those cars shorter, but it lead to occasional collisions.

#### OutsideOfTheRoadPenalty

This cost function takes left and right boundaries of the lane (or lanes), and returns 1 whenever D position fails outside of the boundaries. Otherwise it is 0. This works pretty well, potentially smoothed boundaries would work better. Safe boundaries for the lane are calculated with `GetSafeLaneOffsets`, and basically it is 0.8 meters from left or right of the lane.

## Known Problems and Limitations

1. Because of the way how convertion from Frenet to Cartesian is implemented (smoothing), for the same (x, y), the simulator and the model would have different s, d values. The further from the center of the lane, and the higher curvative in that area, the bigger the difference. It means that simulator might think that car is outside the lane, while visually in UI it is inside.
1. Other cars are assumed to have constant velocity, which means that the model can underestimate their positions, and may end up too close to other cars.
1. The other cars model is simplistic and does not check if other cars would be able to follow their predicted trajectories. One example where it creates a problem is when two cars are following one after another in front of us, and we follow the last one. Then if first car is decelerating, and second is not (yet), in our world model, second car can "jump" over the first one, and because we are following it, our model will assume that distance is good enough, and will not decelerate. Also when there is no "first" car at all, our model can assume that the road at that future time point is free, and could start accelerating.
