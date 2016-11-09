//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//=============================================================================

#include "gauss_seidel.h"
#include "fluid2d.h"
#include "Utilities/Array2T.h"

// Problem 1
void ExSolvePoisson(int _xRes, int _yRes, int _iterations, double _accuracy, Array2d &_field, Array2d &_b) {
	double dx = 1.0 / _xRes;

	for (int i = 0; i < _iterations; i++) {

		for (int y = 1; y < _yRes - 1; y++) {
			for (int x = 1; x < _xRes - 1; x++) {

				double r = _b(x, y) * dx * dx;
				r += _field(x - 1, y);
				r += _field(x + 1, y);
				r += _field(x, y - 1);
				r += _field(x, y + 1);

				_field(x, y) = r / 4.0;
			}
		}

		double totalError = 0.0;

		for (int y = 1; y < _yRes - 1; y++) {
			for (int x = 1; x < _xRes - 1; x++) {

				double approx = (4 * _field(x, y));
				approx -= _field(x - 1, y);
				approx -= _field(x + 1, y);
				approx -= _field(x, y + 1);
				approx -= _field(x, y - 1);
				approx /= dx * dx;

				totalError += std::abs(approx - _b(x, y));
			}
		}

		double avgError = totalError / (double) ((_xRes - 2) * (_yRes - 2));

		if (i == _iterations - 1) {
			printf("Pressure solver: it=%d , res=%f \n", i, avgError);
		}
		else if (avgError < _accuracy) {
			printf("Pressure solver: it=%d , res=%f, converged \n", i, avgError);
			break;
		}

	}
}

// Problem 2
void ExCorrectVelocities(int _xRes, int _yRes, double _dt, const Array2d &_pressure, Array2d &_xVelocity, Array2d &_yVelocity) {

	double dx = 1.0 / _xRes;

	for (int y = 1; y < _yRes - 1; y++) {
		for (int x = 1; x < _xRes - 1; x++) {
			double xVelo = _xVelocity(x + 1, y);
			xVelo -= _dt * (_pressure(x + 1, y) - _pressure(x, y)) / dx;
			_xVelocity(x + 1, y) = xVelo;
		}
	}

	for (int y = 1; y < _yRes - 1; y++) {
		for (int x = 1; x < _xRes - 1; x++) {
			double yVelo = _yVelocity(x, y + 1);
			yVelo -= _dt * (_pressure(x, y + 1) - _pressure(x, y)) / dx;
			_yVelocity(x, y + 1) = yVelo;
		}
	}
}

// Problem 3

double ExAdvectValue(int _x, int _y, double _offsetX, double _offsetY, Array2d &_xVelocity, Array2d &_yVelocity, Array2d &_values, int _xRes, int _yRes, double _dt) {

	double interpolVeloX = ((1.0 - _offsetX) * _xVelocity(_x, _y)) + (_offsetX * _xVelocity(_x + 1, _y));
	double interpolVeloY = ((1.0 - _offsetY) * _yVelocity(_x, _y)) + (_offsetY * _yVelocity(_x, _y + 1));

	double pX = (double)_x - (interpolVeloX * _dt) * (double)_xRes;
	double pY = (double)_y - (interpolVeloY * _dt) * (double)_yRes;

	int interpolBaseX = std::floor(pX);
	int interpolBaseY = std::floor(pY);

	double interpolOffsetX = pX - (double)interpolBaseX;
	double interpolOffsetY = pY - (double)interpolBaseY;

	if (interpolBaseX < 2) {
		interpolBaseX = 0;
		interpolOffsetX = 0.0;
	}
	if (interpolBaseX > _xRes - 3) {
		interpolBaseX = _xRes - 3;
		interpolOffsetX = 0.0;
	}
	if (interpolBaseY < 2) {
		interpolBaseY = 0;
		interpolOffsetY = 0.0;
	}
	if (interpolBaseY > _yRes - 3) {
		interpolBaseY = _yRes - 3;
		interpolOffsetY = 0.0;
	}

	double val = _values(interpolBaseX, interpolBaseY) * (1.0 - interpolOffsetX) * (1.0 - interpolOffsetY);
	val += _values(interpolBaseX + 1, interpolBaseY) * interpolOffsetX * (1.0 - interpolOffsetY);
	val += _values(interpolBaseX, interpolBaseY + 1) * (1.0 - interpolOffsetX) * interpolOffsetY;
	val += _values(interpolBaseX + 1, interpolBaseY + 1) * interpolOffsetX * interpolOffsetY;
	return val;
}

void ExAdvectWithSemiLagrange(int _xRes, int _yRes, double _dt, Array2d &_xVelocity, Array2d &_yVelocity, Array2d &_density, Array2d &_densityTemp, Array2d &_xVelocityTemp, Array2d &_yVelocityTemp) {

	for (int y = 1; y < _yRes - 1; y++) {
		for (int x = 1; x < _xRes; x++) {
			_xVelocityTemp(x, y) = ExAdvectValue(x, y, 0.0, 0.5, _xVelocity, _yVelocity, _xVelocity, _xRes, _yRes, _dt);
		}
	}
	for (int y = 1; y < _yRes; y++) {
		for (int x = 1; x < _xRes - 1; x++) {
			_yVelocityTemp(x, y) = ExAdvectValue(x, y, 0.5, 0.0, _xVelocity, _yVelocity, _yVelocity, _xRes, _yRes, _dt);
		}
	}
	for (int y = 1; y < _yRes - 1; y++) {
		for (int x = 1; x < _xRes - 1; x++) {
			_densityTemp(x, y) = ExAdvectValue(x, y, 0.5, 0.5, _xVelocity, _yVelocity, _density, _xRes, _yRes, _dt);
		}
	}

	for (int y = 1; y < _yRes - 1; y++) {
		for (int x = 1; x < _xRes; x++) {
			_xVelocity(x, y) = _xVelocityTemp(x, y);
		}
	}
	for (int y = 1; y < _yRes; y++) {
		for (int x = 1; x < _xRes - 1; x++) {
			_yVelocity(x, y) = _yVelocityTemp(x, y);
		}
	}
	for (int y = 1; y < _yRes - 1; y++) {
		for (int x = 1; x < _xRes - 1; x++) {
			_density(x, y) = _densityTemp(x, y);
		}
	}
}
