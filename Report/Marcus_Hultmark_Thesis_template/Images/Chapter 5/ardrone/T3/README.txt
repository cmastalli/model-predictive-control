Trajectory description

This trajectory is the horizontal square of 2 m of width, but this time the initial position of the quadrotor is already at 2 meters (the setpoint) in the Z coordinate. This is made to verify the influence of the coupling of the states in the quadrotor model, and even further to analyze this situation in the real quadrotor. 

The noise conditions are simulated with a random number generator taking as a seed value the current time, therefore the seed is never repeated. This noise is restrained to the [-1, 1] range, and is applied to all the states. 
