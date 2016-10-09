//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//=============================================================================

#include "Utilities/Vector2T.h"

// Gravitational acceleration (9.81 m/s^2)
static const double g = 9.81;


// functions for function AdvanceTimeStep1
double springForce1Di(double k, double L, double pj, double pi)
{
    return -k * ((pj-pi) - (-L) ); // * (pj-pi) / fabs(pj-pi);
}

double dampingForce1Di(double d, double v)
{
    return -d * v;
}

// Exercise 1
// Hanging mass point
void AdvanceTimeStep1(double k, double m, double d, double L, double dt, int method, double p1, double v1, double& p2, double& v2)
{
	// Remark: The parameter 'dt' is the duration of the time step, unless the analytic 
	//         solution is requested, in which case it is the absolute time.
    
    
    double springForce2 = springForce1Di(k,L,p2,p1);
    double dampingForce2 = dampingForce1Di(d, v2);
    
    // explicit Euler
    if(method == 1)
    {
        p2 = p2 + dt * v2;
        v2 = v2 + dt * (springForce2 + dampingForce2 + m * g ) / m;
    }
    
    // Symplectic Euler
    if(method == 2)
    {
        v2 = v2 + dt * (springForce2 + dampingForce2 + m * g ) / m;
        p2 = p2 + dt * v2;
    }
    
    // explicit midpoint
    if(method == 3)
    {
        double p2Half = p2 + dt / 2.0 * v2;
        double v2Half = v2 + dt / 2.0 * (springForce2 + dampingForce2 + m * g) / m;
        
        double springForceHalf = springForce1Di(k, L, p2Half, p1);
        double dampingForceHalf = dampingForce1Di(d, v2Half);
        
        p2 = p2 + dt * v2Half;
        v2 = v2 + dt * (springForceHalf + dampingForceHalf + m * g) / m;
    }
    
    // Semi-Implicit Euler
    if(method == 4)
    {
        double dforcedx = -k;
        double dforcedv = -d;
        v2 = ((m - dt * dforcedv) * v2 + dt * (springForce2 + dampingForce2 + m * g)) / (m - dt * dforcedv - dt * dt * dforcedx );
        p2 = p2 + dt * v2;
    }
    
    // analytic solution
    if(method == 5)
    {
        double alpha = - d / (2.0 * m);
        double beta = sqrt( 4.0 * k * m - d * d ) / (2.0 * m);
        double c1 = m * g / k;
        double c2 = - c1 * alpha / beta;
        
        p2 = c1 * exp(alpha * dt) * cos(beta * dt) + c2 * exp(alpha * dt) * sin(beta * dt) - L - c1;
        v2 = c1 * exp(alpha * dt) * (alpha * cos(beta * dt) - beta * sin(beta * dt)) + c2 * exp(alpha * dt) * (alpha * sin(beta * dt) + beta * cos(beta * dt));
    }
    
}

// functions for function AdvanceTimeStep3
Vec2 springForce2Di(double k, double L, Vec2 pj, Vec2 pi)
{
    return - k * ( (pj - pi).norm() - L ) / (pj - pi).norm() * (pj - pi);
}

Vec2 dampingForce2Di(double d, Vec2 v)
{
    return - d * v;
}

// Exercise 3
// Falling triangle
void AdvanceTimeStep3(double k, double m, double d, double L, double dt,
                      Vec2& p1, Vec2& v1, Vec2& p2, Vec2& v2, Vec2& p3, Vec2& v3)
{
	// p1 += Vec2(1,1);
    
    // use symplectic euler method
    
    // stiffness for ground force
    double kr = 100.0;
    
    Vec2 springForce12 = springForce2Di(k, L, p1, p2);
    Vec2 springForce13 = springForce2Di(k, L, p1, p3);
    Vec2 springForce23 = springForce2Di(k, L, p2, p3);
    
    Vec2 dampingForce1 = dampingForce2Di(d, v1);
    Vec2 dampingForce2 = dampingForce2Di(d, v2);
    Vec2 dampingForce3 = dampingForce2Di(d, v3);
    
    
    Vec2 force1 = springForce12 + springForce13 + dampingForce1 - Vec2(0.0, m * g);
    
    if(p1.y() <= -1.0)
    {
        // force induced by ground
        Vec2 forceGround1 = -springForce2Di(kr, L, p1, Vec2(p1.x(),-1.0));
        force1 += forceGround1;
    }
    
    v1 = v1 + dt * force1 / m;
    p1 = p1 + dt * v1;
    
    
    Vec2 force2 = -springForce12 + springForce23 + dampingForce2 - Vec2(0.0, m * g);
    
    if(p2.y() <= -1.0)
    {
        Vec2 forceGround2 = -springForce2Di(kr, L, p2, Vec2(p2.x(),-1.0));
        force2 += forceGround2;
    }
    
    v2 = v2 + dt * force2 / m; // force / m;
    p2 = p2 + dt * v2;
    
    
    Vec2 force3 = -springForce13 - springForce23 + dampingForce3 - Vec2(0.0, m * g);
    
    if(p3.y() <= -1.0)
    {
        Vec2 forceGround3 = -springForce2Di(kr, L, p3, Vec2(p3.x(),-1.0));
        force3 += forceGround3;
    }
    
    v3 = v3 + dt * force3 / m;
    p3 = p3 + dt * v3;
    
    
    
    
}
