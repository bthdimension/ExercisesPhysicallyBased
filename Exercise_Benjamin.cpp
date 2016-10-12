//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//=============================================================================

#include "Utilities/Vector2T.h"

// Gravitational acceleration (9.81 m/s^2)
static const double g = 9.81;

double totalForce1D(double k, double m, double d, double L, double p1, double p2, double v2)
{
	double currentLength = (p1 - p2);
	double internalForce = k*(currentLength - L) ;	
	double externalForce = -m * g;													// pulls p2 down, needs to be negative
	double dampingForce = -d*v2;
	return (externalForce + internalForce + dampingForce);
}


// Exercise 1
// Hanging mass point
void AdvanceTimeStep1(double k, double m, double d, double L, double dt, int method, double p1, double v1, double& p2, double& v2)
{
	// Remark: The parameter 'dt' is the duration of the time step, unless the analytic 
	//         solution is requested, in which case it is the absolute time.
	// k = stiffness
	// m = mass
	// d = damping
	// L = Initial Length (p1 - p2)
	// dt = duration of the time step
	// method (explicit euler, symplectic euler, midpoint, backwards_euler, analytic)
	// p1 / p2 = point 1 / 2
	// v1 / v2 = velocity 1 / 2

	double totalForce = totalForce1D(k, m, d, L, p1, p2, v2);

	double derivativeOfVelocity = totalForce / m;

	if (method == 1)
	{
		p2 = p2 + dt*v2;

		v2 = v2 + dt*derivativeOfVelocity;
	}
	else if (method == 2)
	{
		v2 = v2 + dt*derivativeOfVelocity;

		p2 = p2 + dt*v2;
	}
	else if (method == 3)
	{
		double velocityHalf = v2 + (dt / 2.0)*derivativeOfVelocity;
		double positionHalf = p2 + (dt / 2.0)*v2;

		p2 = p2 + dt*velocityHalf;

		double totalForceHalf = totalForce1D(k, m, d, L, p1, positionHalf, velocityHalf);
		double derivativeOfVelocityHalf = totalForceHalf / m;
		v2 = v2 + dt*derivativeOfVelocityHalf;
	}
	else if (method == 4)
	{
		double dFp = -k;	// derivative of the force with respect to position
		double dFv = -d;	// derivative of the force with respect to velocity

		double b = (m - dt*dFv)*v2 + dt*totalForce;
		double A = m - dt*dFv - dt*dt*dFp;
		v2 = b / A;

		p2 = p2 + dt*v2;
	}
	else if (method == 5)
	{
		double alpha = -d / (2*m);
		double beta = sqrt(4 * k*m - d*d) / (2 * m);
		double c1 = m*g / k;
		double c2 = -alpha*c1 / beta;

		p2 = c1*exp(alpha*dt)*cos(beta*dt) + c2*exp(alpha*dt)*sin(beta*dt) - L - (m*g / k);
		v2 = alpha*c1*exp(alpha*dt)*cos(beta*dt) - beta*c1*exp(alpha*dt)*sin(beta*dt)
			+ alpha*c2*exp(alpha*dt)*sin(beta*dt) + beta*c2*exp(alpha*dt)*cos(beta*dt);
	}
}

Vec2 totalForce2D(double k, double m, double d, double L,
	Vec2& p, Vec2& v, Vec2& pi, Vec2& pj)
{
	int floorPos = -1;
	double kr = 100;
	double lengthi = (p - pi).norm();
	double lengthj = (p - pj).norm();

	Vec2 internalForcei = -k*(lengthi - L) * (p - pi)/lengthi;
	Vec2 internalForcej = -k*(lengthj - L) * (p - pj) / lengthj;

	Vec2 externalForce = Vec2(0, -m * g);
	Vec2 dampingForce = -d*v;

	Vec2 totalForce = internalForcei + internalForcej + externalForce + dampingForce;

	if (p.y() < floorPos)
	{
		totalForce += Vec2(0, -kr*(p.y()-floorPos));
	}
	return totalForce;
}

Vec2 derivativeOfVelocity(Vec2& totalForce, double m)
{
	return totalForce / m;
}

// Exercise 3
// Falling triangle
void AdvanceTimeStep3(double k, double m, double d, double L, double dt,
	Vec2& p1, Vec2& v1, Vec2& p2, Vec2& v2, Vec2& p3, Vec2& v3)
{
	Vec2 nv1 = v1 + dt*derivativeOfVelocity(totalForce2D(k, m, d, L, p1, v1, p2, p3), m); // created placeholder nv1 so I don't already use new v1 to calculate v2 but not the other way around
	Vec2 nv2 = v2 + dt*derivativeOfVelocity(totalForce2D(k, m, d, L, p2, v2, p1, p3), m);
	v3 = v3 + dt*derivativeOfVelocity(totalForce2D(k, m, d, L, p3, v3, p1, p2), m);
	v1 = nv1;
	v2 = nv2;

	p1 = p1 + dt*v1;
	p2 = p2 + dt*v2;
	p3 = p3 + dt*v3;
}
