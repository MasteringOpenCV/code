/*****************************************************************************
*   Ch9 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*   http://code.google.com/p/fluidwall/
*****************************************************************************/
/**
 * @file      FluidSolver.cpp
 * @author    Austin Hines <futurelightstudios@gmail.com>
 * @copyright 2011 Austin Hines, Naureen Mahmood, and Texas A&M Dept. of Visualization
 * @version	  1.0.0
 * 
 * This file is part of Fluid Wall. You can redistribute it and/or modify            
 * it under the terms of the GNU Lesser General Public License as published  
 * by the Free Software Foundation, either version 3 of the License, or     
 * (at your option) any later version.                                       
 *                                                                             
 * Fluid Wall is distributed in the hope that it will be useful,                 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of            
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              
 * GNU Lesser General Public License for more details.                       
 *                                                                            
 * You should have received a copy of the GNU Lesser General Public License  
 * along with Fluid Wall. If not, see <http://www.gnu.org/licenses/>.            
 *
 */

#include "FluidSolver.h"
#include <stdlib.h>
#include <stdio.h>


#define ROW_WIDTH N_+2
#define IX(i,j) ((i)+(ROW_WIDTH)*(j))
#define FOR_EACH_CELL for (i=1 ; i<=N_ ; i++) { for (j=1 ; j<=N_ ; j++) {
#define END_FOR }}
#define SWAP(x0,x) { float* tmp=x0; x0=x; x=tmp; }



FluidSolver::FluidSolver(void)
{
	FluidSolver(128, 0.1f, 0.00f, 0.0f);
}



FluidSolver::FluidSolver(int N, float dt, float diff, float visc)
{
	N_      = N;
	dt_     = dt;
	diff_   = diff;
	visc_   = visc;

	int size = getSize();

	u_			= (float *) malloc(size * sizeof(float));
	v_			= (float *) malloc(size * sizeof(float));
	u_prev_		= (float *) malloc(size * sizeof(float));
	v_prev_		= (float *) malloc(size * sizeof(float));
	dens_		= (float *) malloc(size * sizeof(float));	
	dens_prev_	= (float *) malloc(size * sizeof(float));
	bounds_	    = (bool  *) malloc(size * sizeof(bool) ); 

}



FluidSolver::~FluidSolver(void)
{
	if ( u_ ) free ( u_ );
	if ( v_ ) free ( v_ );
	if ( u_prev_ ) free ( u_prev_ );
	if ( v_prev_ ) free ( v_prev_ );
	if ( dens_ ) free ( dens_ );
	if ( bounds_ ) free ( bounds_ );
	if ( dens_prev_ ) free ( dens_prev_ );
}



void FluidSolver::reset()
{
	for (int i=0 ; i < getSize() ; i++) {
		u_[i] = v_[i] = u_prev_[i] = v_prev_[i] = dens_[i] = dens_prev_[i] = 0.0f;
		bounds_[i] = false;
	}
}



void FluidSolver::update()
{
	computeDensityStep(dens_, dens_prev_, u_, v_);
	computeVelocityStep(u_, v_, u_prev_, v_prev_);

	//reset u_prev_, v_prev_, and dens_prev
	for (int i=0 ; i < getSize() ; i++) 
		u_prev_[i] = v_prev_[i] = dens_prev_[i] = 0.0f;
}



//TODO: Can increase efficiency by only testing valid coordinates 
//      when emitters are created.
void FluidSolver::addVertVelocityAt(int x, int y, float value)
{
	if(isValidCoordinate(x, y))
		v_prev_[IX(x,y)] += value;
}



void FluidSolver::addHorzVelocityAt(int x, int y, float value)
{
	if(isValidCoordinate(x, y))
		u_prev_[IX(x,y)] += value;
}



void FluidSolver::addDensityAt(int x, int y, float value)
{
	if(isValidCoordinate(x, y))
		dens_prev_[IX(x,y)] += value;
}



void FluidSolver::setBoundAt(int x, int y, bool isBound)
{
	if(isValidCoordinate(x, y))
		bounds_[IX(x,y)] = isBound;
}



//accessors
bool FluidSolver::isBoundAt(int x, int y)
{
	return bounds_[IX(x,y)];
}



float FluidSolver::getDensityAt(int x, int y)
{
	return dens_[IX(x,y)];
}



float FluidSolver::getVertVelocityAt(int x, int y)
{
	return v_[IX(x,y)];
}



float FluidSolver::getHorzVelocityAt(int x, int y)
{
	return u_[IX(x,y)];
}



///protected functions
int FluidSolver::getSize()
{
	return (ROW_WIDTH) * (ROW_WIDTH);
}



bool FluidSolver::isValidCoordinate(int x, int y)
{
	bool xIsValid = (x >= 1) && (x <= N_);
	bool yIsValid = (y >= 1) && (y <= N_);

	if(xIsValid && yIsValid)
		return true;
	else
		return false;
}



void FluidSolver::addSource(float* x, float* s)
{
	int i;
	for (i=0 ; i<getSize() ; i++) 
		x[i] += dt_*s[i];
}



void FluidSolver::setBounds(int boundsFlag, float* x)
{
	int i, j;

	//free slip boundary edges
	for ( i=1 ; i<=N_; i++ ) {
		//reverse velocity component on vertical walls (u)
		x[IX(0  ,i)]  = boundsFlag==1 ? -x[IX(1,i)] : x[IX(1,i)];
		x[IX(N_+1,i)] = boundsFlag==1 ? -x[IX(N_,i)] : x[IX(N_,i)];
		
		//reverse velocity component on horizontal (top and bottom) walls (v)
		x[IX(i,0  )]  = boundsFlag==2 ? -x[IX(i,1)] : x[IX(i,1)];
		x[IX(i,N_+1)] = boundsFlag==2 ? -x[IX(i,N_)] : x[IX(i,N_)];
	}
	
	FOR_EACH_CELL
		//detect bounds changes for each cell and reverse velocity components
		//set current value to a negative average of surrounding values
		if(bounds_[IX(i,j)]) {
			//set current value to a negative average of surrounding values
			x[IX(i,j)] = 0;

			//vertical (v)
			//if bounds are about to turn off, velocity component of off cell becomes positive.
			if(!bounds_[IX(i,j+1)]) 
				x[IX(i,j)] = boundsFlag==2 ? -x[IX(i,j+1)] : x[IX(i,j+1)]; 

			//horizontal (u)
			//if bounds change from on to off, velocity component of off cell becomes positive.
			if(!bounds_[IX(i+1,j)]) 
				x[IX(i,j)] = boundsFlag==1 ? -x[IX(i+1,j)] : x[IX(i+1,j)]; 
		}
		else {
			//vertical (v)
			//if bounds change from off to on, velocity component of off cell becomes negative.
			if(bounds_[IX(i,j+1)]) 
				x[IX(i,j+1)] = boundsFlag==2 ? -x[IX(i,j)] : x[IX(i,j)];
			
			//horizontal (u)
			//if bounds change from off to on, velocity component of off cell becomes negative.
			if( bounds_[IX(i+1,j)]) 
				x[IX(i+1,j)] = boundsFlag==1 ? -x[IX(i,j)] : x[IX(i,j)]; 
			
		}
	END_FOR

	//handle corner conditions of objects
	FOR_EACH_CELL
		// X = cell in question | 0 = open cell | * = closed cell (with a boundary)

		// * 0
		// X *
		if(bounds_[IX(i,j)] && bounds_[IX(i+1, j)] && bounds_[IX(i, j+1)] && !bounds_[IX(i+1, j+1)])
			x[IX(i, j)] = 0.5 *(x[IX(i+1, j)] + x[IX(i, j+1)]);
		// X * 
		// * 0
		else if(bounds_[IX(i,j)] && bounds_[IX(i+1, j)] && bounds_[IX(i, j-1)] && !bounds_[IX(i+1, j-1)])
			x[IX(i, j)] = 0.5 *(x[IX(i+1, j)] + x[IX(i, j-1)]);
		// * X 
		// 0 *
		else if(bounds_[IX(i,j)] && bounds_[IX(i-1, j)] && bounds_[IX(i, j-1)] && !bounds_[IX(i-1, j-1)])
			x[IX(i, j)] = 0.5 *(x[IX(i-1, j)] + x[IX(i, j-1)]);
		// 0 * 
		// * X
		else if(bounds_[IX(i,j)] && bounds_[IX(i-1, j)] && bounds_[IX(i, j+1)] && !bounds_[IX(i-1, j+1)])
			x[IX(i, j)] = 0.5 *(x[IX(i-1, j)] + x[IX(i, j+1)]);
	END_FOR

	//corner conditions
	x[IX(0,      0     )] = 0.5f * (x[IX(1,  0   )] + x[IX(0,      1 )]);
	x[IX(0,      N_ + 1)] = 0.5f * (x[IX(1,  N_+1)] + x[IX(0,      N_)]);
	x[IX(N_ + 1, 0     )] = 0.5f * (x[IX(N_, 0   )] + x[IX(N_ + 1, 1 )]);
	x[IX(N_ + 1, N_ + 1)] = 0.5f * (x[IX(N_, N_+1)] + x[IX(N_ + 1, N_)]);
}



void FluidSolver::linearSolve( int boundsFlag, float* x, float* x0, float a, float c)
{
	int i, j, k;

	//use 20 iterations of Gauss-Sidel to find a convergence of values
	for ( k=0 ; k<20 ; k++ ) {
		FOR_EACH_CELL
			//exchange values with neighbors
			x[IX(i,j)] = (x0[IX(i,j)] + a*(x[IX(i-1,j)] + x[IX(i+1,j)] + x[IX(i,j-1)] + x[IX(i,j+1)])) / c;
		END_FOR
		// factor in boundary conditions with each solution iteration
		
		setBounds(boundsFlag, x); 
	}
}



void FluidSolver::diffuse (int boundsFlag, float* x, float* x0)
{
	float diffusionPerCell = dt_ * diff_ * N_ * N_;
	linearSolve ( boundsFlag, x, x0, diffusionPerCell, 1+4*diffusionPerCell);
}



void FluidSolver::advect (int boundsFlag, float* d, float* d0, 
						  float* u, float* v)
{
	int i, j;
	int i0; //new x cell coordinate based on velocity grid
	int j0; //new y cell coordinate based on velocity grid
	int i1; //x + 1 cell next to new cell coordinate
	int j1; //y + 1 cell next to new cell coordinate
	float x, y, s0, t0, s1, t1, dt0;

	//initial time differential = dt * number of cells in a row
	dt0 = dt_ * N_;

	//back trace density and velocity values from the center of each cell
	FOR_EACH_CELL

		// calculate new coordinates based on existing velocity grids
		x = i - dt0 * u[IX(i,j)]; 
		y = j - dt0 * v[IX(i,j)];
		
		//limit x coordinate to fall within the grid 
		if (x < 0.5f)      x = 0.5f; 
		if (x > N_ + 0.5f) x = N_ + 0.5f; 
		i0 = (int)x; 
		i1 = i0 + 1;
		
		//limit y coordinate to fall within the grid
		if (y < 0.5f)      y = 0.5f; 
		if (y > N_ + 0.5f) y = N_ + 0.5f; 
		j0 = (int)y; 
		j1 = j0 + 1;

		s1 = x - i0; //difference between calculated x position and limited x position
		s0 = 1 - s1; //calculate relative x distance from center of this cell

		t1 = y - j0;  
		t0 = 1 - t1;
		
		//blend several values from the velocity grid together, based on where in the cell 
		//the new coordinate valls
		d[IX(i,j)] = s0 * (t0 * d0[IX(i0,j0)] + t1 * d0[IX(i0,j1)]) +
					 s1 * (t0 * d0[IX(i1,j0)] + t1 * d0[IX(i1,j1)]);
	END_FOR
	setBounds(boundsFlag, d);
}



void FluidSolver::project( float* u, float* v, float* p, float* div)
{
	int i, j;

	float h = 1.0 / N_; //calculate unit length of each cell relative to the whole grid.

	FOR_EACH_CELL
		//calculate initial solution to gradient field based on the difference in velocities of
		//surrounding cells.
		div[IX(i, j)] = -0.5f * h * (u[IX(i + 1, j)] - u[IX(i - 1, j)] + v[IX(i, j + 1)] - v[IX(i, j - 1)]);
		//set projected solution values to be zero
		p[IX(i, j)] = 0; 
	END_FOR	

	//set bounds for diffusion
	setBounds(0, div); 
	setBounds(0, p);

	// calculate gradient (height) field
	linearSolve (0, p, div, 1, 4);

	FOR_EACH_CELL
		//subtract gradient field from current velocities
		u[IX(i, j)] -= 0.5f * N_ * (p[IX(i + 1, j)] - p[IX(i - 1, j)]);
		v[IX(i, j)] -= 0.5f * N_ * (p[IX(i, j + 1)] - p[IX(i, j - 1)]);
	END_FOR

	//set boundaries for velocity
	setBounds(1, u); 
	setBounds(2, v);
}



void FluidSolver::computeDensityStep( float* x, float* x0, float* u, float* v )
{
	addSource (x, x0);
	SWAP(x0, x); 
	diffuse(0, x, x0);
	SWAP(x0, x); 
	advect(0, x, x0, u, v);
}



void FluidSolver::computeVelocityStep (float* u, float* v, float* u0, float* v0)
{
	addSource(u, u0); 
	addSource(v, v0);
	//diffuse horizontal 
	SWAP(u0, u); 
	diffuse(1, u, u0);

	//diffuse vertical
	SWAP(v0, v); 
	diffuse(2, v, v0);
	project (u, v, u0, v0);
	SWAP(u0, u); 
	SWAP (v0, v);

	//advect velocities
	advect(1, u, u0, u0, v0); 
	advect(2, v, v0, u0, v0);
	project(u, v, u0, v0);
}
