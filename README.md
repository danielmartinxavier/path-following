# path-following
3D Path-Following algorithms for circular paths using X-Plane

Before launching the simulation, X-Plane 10 should be already launched and ready to use. Then, the script *main_script_X_Plane.m* should be run on MATLAB to start the simulation. In this script, the user can choose the path-following algorithm (Carrot-Chasing, NLGL+, PLOS+, Vector Field) to be used during the simulation. Each strategy is coded in a different script:

- *X_Plane_Carrot.m*
- *X_Plane_NLGL.m*
- *X_Plane_PLOS.m*
- *X_Plane_Vector.m*

They are called by the script *main_script_X_Plane.m* to run the simulation. The parameters for each strategy can be modified using the script *parameters.m*. The *metrics.m* file computes some important metrics of the simulation. 

**Note:** the wind intensity should be set manually on X-Plane and the user is responsible for choosing the airplane type (Cessna 172SP recommended).

**Note 2:** during runtime, there are some errors that may occur due to a communication failure between MATLAB and X-Plane. The code is programmed to deal with this by reinitializing the simulation. That's why a complete simulation can take more time than expected.

### Citing

If you use this project for academic work, please consider citing our [publication](https://link.springer.com/article/10.1007/s10846-022-01764-4): 

Xavier, D.M., Silva, N.B.F. & Branco, K.R.L.J.C. Path-following Algorithms Comparison using Software-in-the-Loop Simulations for UAVs. J Intell Robot Syst 106, 63 (2022). 
