#pragma once

#include "drake/common/trajectories/piecewise_polynomial.h"

class RobotSystem;

struct Traj {
	drake::trajectories::PiecewisePolynomial<double> x;
	drake::trajectories::PiecewisePolynomial<double> u;
};

void playTrajectory(drake::trajectories::PiecewisePolynomial<double>& stateTraj, RobotSystem& sys, double rate = 1.0);