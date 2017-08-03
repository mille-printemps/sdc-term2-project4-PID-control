# How it works

## For tuning the coefficients

1. Set initial values to the variables `K_P`, `K_I` and `K_D` in `main.cpp`. Those variables were set to `0.1`, `0.0001` and `4.0`, respectively for this project. 
2. Change the variable `twiddling` to `true` in `main.cpp`.
3. Re-compile the program and run it with the simulator. 
4. 'Twiddling' will start and the vehicle in the simulator will start running the course after the 'twiddling' satisfies the `THRESHOLD` defined in `main.cpp`.

## For driving the vehicle with the simulator

1. Set the coefficients adjusted by the tuning above to the variables `K_P`, `K_I` and `K_D` in `main.cpp`.  Those variables were adjusted to `0.3079`, `6.561e-05` and `4.0` as a result of the 'twiddling'.
2. Change the variable `twiddling` to `false` in `main.cpp`.
3. Re-compile the program and run it with the simulator. The vehicle will just run the course. 

# PID components

As a result of the tuning of the coefficients, `D` component had the largest effect, `P` components was the second and `I` component was the last to the control. In terms of the magnitude of the coefficients, the effect of `D` component was about 10 times as large as `P` component and the effect of `I` component occupied a small portion among them.

# How the coefficients were tuned

The 'twiddling' algorithm was implemented and used. The algorithm is implemented in `PID.cpp`. It worked for me, but under some conditions. 

1. **Appropriate initial values of the steps:** the initial values of the steps (the initial values of the variable `dp` in the lecture) would have to be set to values close to the ones that may be resulting values. Setting these initial values would require knowledge, experience and trial-and-error. In this project, these values were decided by trial-end-error. 
2. **Appropriate value of the threshold:** in the 'twiddling' algorithm, the threshold to finish the algorithm is supposed to be a small value such as `0.001`. However, for this project, that magnitude of the value would be too small because the algorithm would take an unrealistically long time to finish. In the implementation, the threshold is decided depending on the magnitude of the initial value of the steps. Refer to `main.cpp` for this implementation.
3. **Appropriate number of samples:** in the lecture, `200` samples were used as samples for 'twiddling'. However, this number would be too small for this project. In the implementation, `1000` samples were picked up and the first `100` samples were discarded as done in the lecture, then the rest `900` samples were used for 'twiddling'. Refer to `main.cpp` for this setting. Of course, more samples could be used with a trade-off of performance (time for computing, precision of the coefficients and etc.)

In summary, the coefficients were tuned by combination of manual tuning and the 'twiddling' algorithm.

