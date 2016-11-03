//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//=============================================================================

#include "gauss_seidel.h"
#include "fluid2d.h"
#include "Utilities/Array2T.h"

// Problem 1
void ExSolvePoisson(int _xRes, int _yRes, int _iterations, double _accuracy, Array2d &_field, Array2d &_b)
{
    
	double dx = 1.0 / _xRes;
	
    for (int n = 0; n < _iterations; n++)
    {
        double residual = 0.0;
        
        // Note that the boundaries are handles by the framework, so you iterations should be similar to:
        for (int y = 1; y < _yRes - 1; y++)
        {
            for (int x = 1; x < _xRes - 1; x++)
            {
                //double temp = _field(x,y);
                residual += std::abs( _b(x, y) - (4 * _field(x, y) - _field(x + 1, y) - _field(x, y + 1) - _field(x - 1, y) - _field(x, y - 1) ) / (dx * dx) );
                
                _field(x,y) = (dx * dx * _b(x, y) + _field(x - 1, y) + _field(x, y - 1) + _field(x + 1, y) + _field(x, y + 1) ) / 4.0;
                
            }
        }
        
        residual /= (double)(_xRes * _yRes);
        
        // For your debugging, and ours, please add these prints after every iteration
        if(n == _iterations - 1)
        	printf("Pressure solver: it=%d , res=%f \n", n, residual);
        if(residual < _accuracy)
        {
            printf("Pressure solver: it=%d , res=%f, converged \n", n, residual);
            break;
        }
    }
    
}

// Problem 2
void ExCorrectVelocities(int _xRes, int _yRes, double _dt, const Array2d &_pressure, Array2d &_xVelocity, Array2d &_yVelocity)
{
	double dx = 1.0 / _xRes;

	// Note: velocity u_{i+1/2} is practically stored at i+1
    
    for (int y = 1; y < _yRes - 1; y++)
    {
        for (int x = 1; x < _xRes - 1; x++)
        {
            _xVelocity(x + 1, y) = _xVelocity(x + 1,y) - _dt/dx * (_pressure(x + 1,y) - _pressure(x,y));
            _yVelocity(x, y + 1) = _yVelocity(x,y + 1) - _dt/dx * (_pressure(x,y + 1) - _pressure(x,y));
        }
    }

}

double linearInterpolation(double y1, double y2, double dx, double dxp)
{
    
    return (y2 - y1) / dx * dxp + y1;
}


double bilinearInterpolation(double d11, double d21, double d12, double d22, double dx, double dxp, double dyp)
{
    double dp1 = linearInterpolation(d11, d21, dx, dxp);
    double dp2 = linearInterpolation(d12, d22, dx, dxp);
    
    return linearInterpolation(dp1, dp2, dx, dyp);
}

// Problem 3
void ExAdvectWithSemiLagrange(int _xRes, int _yRes, double _dt, Array2d &_xVelocity, Array2d &_yVelocity, Array2d &_density, Array2d &_densityTemp, Array2d &_xVelocityTemp, Array2d &_yVelocityTemp)
{
	// Note: velocity u_{i+1/2} is practically stored at i+1
    
    for (int y = 1; y < _yRes - 1; y++)
    {
        for (int x = 1; x < _xRes - 1; x++)
        {
            // interpolation between u(i - 0.5,j) and u(i + 0.5,j) to get u(i,j), same for v(i,j)
            // uG = (uGx, uGy)
            double uGx = ( _xVelocity(x + 1, y) + _xVelocity(x, y) ) / 2.0; // + _xVelocity(x - 1, y)
            double uGy = ( _yVelocity(x, y + 1) + _yVelocity(x, y) ) / 2.0; // + _yVelocity(x, y - 1)
            
            
            // backwards tracing to get xP = xG - dt*uG
            // xG = (xGx, xGy)
            double xGx = double(x);
            double xGy = double(y);
            // xP = (xPx, xPy)
            double xPx = xGx - _dt * uGx;
            double xPy = xGy - _dt * uGy;
            
            // bilinear interpolation for different positions of xP
            if(xPx > xGx && xPy > xGy)
            {
                
                double dxp = xPx - xGx;
                double dyp = xPy - xGy;
                
                _densityTemp(x,y)   = bilinearInterpolation(  _density(x,y),   _density(x+1,y),   _density(x,y+1),   _density(x+1,y+1), 1.0, dxp, dyp);
                _xVelocityTemp(x,y) = bilinearInterpolation(_xVelocity(x,y), _xVelocity(x+1,y), _xVelocity(x,y+1), _xVelocity(x+1,y+1), 1.0, dxp, dyp);
                _yVelocityTemp(x,y) = bilinearInterpolation(_yVelocity(x,y), _yVelocity(x+1,y), _yVelocity(x,y+1), _yVelocity(x+1,y+1), 1.0, dxp, dyp);
                
                
            }
            else if(xPx < xGx && xPy > xGy)
            {
                
                double dxp = xPx - (xGx - 1.0);
                double dyp = xPy - xGy;
                
                _densityTemp(x,y)   = bilinearInterpolation(  _density(x-1,y),   _density(x,y),   _density(x-1,y+1),   _density(x,y+1), 1.0, dxp, dyp);
                _xVelocityTemp(x,y) = bilinearInterpolation(_xVelocity(x-1,y), _xVelocity(x,y), _xVelocity(x-1,y+1), _xVelocity(x,y+1), 1.0, dxp, dyp);
                _yVelocityTemp(x,y) = bilinearInterpolation(_yVelocity(x-1,y), _yVelocity(x,y), _yVelocity(x-1,y+1), _yVelocity(x,y+1), 1.0, dxp, dyp);
            }
            else if(xPx < xGx && xPy < xGy)
            {
                
                double dxp = xPx - (xGx - 1.0);
                double dyp = xPy - (xGy - 1.0);
                
                _densityTemp(x,y)   = bilinearInterpolation(  _density(x-1,y-1),   _density(x,y-1),   _density(x-1,y),   _density(x,y), 1.0, dxp, dyp);
                _xVelocityTemp(x,y) = bilinearInterpolation(_xVelocity(x-1,y-1), _xVelocity(x,y-1), _xVelocity(x-1,y), _xVelocity(x,y), 1.0, dxp, dyp);
                _yVelocityTemp(x,y) = bilinearInterpolation(_yVelocity(x-1,y-1), _yVelocity(x,y-1), _yVelocity(x-1,y), _yVelocity(x,y), 1.0, dxp, dyp);
            }
            else if(xPx > xGx && xPy < xGy)
            {
                
                double dxp = xPx - xGx;
                double dyp = xPy - (xGy - 1.0);
                
                
                _densityTemp(x,y)   = bilinearInterpolation(  _density(x,y-1),   _density(x+1,y-1),   _density(x,y),   _density(x+1,y), 1.0, dxp, dyp);
                _xVelocityTemp(x,y) = bilinearInterpolation(_xVelocity(x,y-1), _xVelocity(x+1,y-1), _xVelocity(x,y), _xVelocity(x+1,y), 1.0, dxp, dyp);
                _yVelocityTemp(x,y) = bilinearInterpolation(_yVelocity(x,y-1), _yVelocity(x+1,y-1), _yVelocity(x,y), _yVelocity(x+1,y), 1.0, dxp, dyp);
            }

        }
        
    }
    
    _density = _densityTemp;
    _xVelocity = _xVelocityTemp;
    _yVelocity = _yVelocityTemp;
    
    
}


















