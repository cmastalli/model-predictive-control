lbmpc_qpoases
=============

This is an implementation of Learning-Based Model Predictive Control (LBMPC) that uses
the qpOASES dense solver.

Prerequisites
=============
* [qpOASES](http://www.kuleuven.be/optec/software/qpOASES)
* [Eigen3](http://eigen.tuxfamily.org/)
* [CMake](http://www.cmake.org/)

Compiling examples
==================

    cd lbmpc_qpoases
    mkdir build
    cd build
    cmake ..
    export N_MPC_STEPS=15 # or whatever..
    make

Creating data files
===================

Example from documentation
--------------------------
(in PYTHON):

    >> cd model-predictive-control/mpc/script
    >> python lbmpc_control_design.py

Prerequisites
=============
* [python-control](http://python-control.sourceforge.net/manual/index.html)
* [Slycot](https://github.com/avventi/Slycot)
* [PyYAML](http://pyyaml.org/wiki/PyYAMLDocumentation)
* [CVXOPT](http://cvxopt.org/install/index.html) for ATLAS installation: (http://sciruby.com/docs/installation/atlas.html)

Quadrotor example
-----------------
(in MATLAB):

    >> cd lbmpc/matlab/qr_example
    >> Init

How to run examples
===================

Example from documentation
--------------------------
    cd lbmpc_lssol
    build/bin/example0 matlab/example0/ConstrParam.bin

Quadrotor example
-----------------
	cd lbmpc_lssol
	build/bin/qr_example matlab/qr_example/quad.bin

[Back to home](https://bitbucket.org/lbmpc/lbmpc.bitbucket.org/wiki/Home)
