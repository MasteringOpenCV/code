/*****************************************************************************
*   Ch9 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*   http://code.google.com/p/fluidwall/
*****************************************************************************/
/**
 * @file      FluidSolver.h
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

#pragma once
#include <vector>

using namespace std;

/**
 * A 2D fluid simulation solver based on the Stable Fluids algorithm.
 *
 * This algorithm based on original C code from Jos Stam's 2001 paper, "Real Time Fluid
 * Dynamics for Games." I have adapted this code to C++ style and classes (vectors instead
 * of arrays, etc.) 
 * 
 */
class FluidSolver
{
public:
	//TODO: change all comments to reflect the switch to vectors.

	/**
	 * Default constructor.
	 * 
	 */
	FluidSolver(void);

	/**
	 * Parameter constructor.
	 * @param N      Width (and height) of the square fluid simulation grid
	 * @param dt     Timestep size
	 * @param diff   Diffusion coefficient
	 * @param visc   Viscosity coefficient
	 */
	FluidSolver(int N, float dt, float diff, float visc);
	~FluidSolver(void);

	/**
	 * Adds vertical velocity values at a particular coordinate. 
	 *
	 * The system is designed to add values from multiple other sources using this method, 
	 * and then commit them to the simulation using update(). Valid indicies range from 
	 * 1 to N. Indicies 0 and N+1 are buffer rows for algorithms.
	 *
	 * @param x     x-coordinate, valid values: 1 - N
	 * @param y     y-coordinate, valid values: 1 - N
	 * @param value New vertical velocity value
	 */
	void addVertVelocityAt(int x, int y, float value);


	/**
	 * Adds horizontal velocity values at a particular coordinate. 
	 *
	 * The system is designed to add values from multiple other sources using this method, 
	 * and then commit them to the simulation using update(). Valid indicies range from 
	 * 1 to N. Indicies 0 and N+1 are buffer rows for algorithms.
	 *
	 * @param x     x-coordinate, valid values: 1 - N
	 * @param y     y-coordinate, valid values: 1 - N
	 * @param value New horizontal velocity value
	 */
	void addHorzVelocityAt(int x, int y, float value);


	/**
	 * Adds density values at a particular coordinate. 
	 *
	 * The system is designed to add values from multiple other sources using this method, 
	 * and then commit them to the simulation using update(). Valid indicies range from 
	 * 1 to N. Indicies 0 and N+1 are buffer rows for algorithms.
	 *
	 * @param x     x-coordinate, valid values: 1 - N
	 * @param y     y-coordinate, valid values: 1 - N
	 * @param value Density value to be added
	 */
	void addDensityAt(int x, int y, float value);

	/**
	 * Sets bound condition at a given cell.
	 *
	 * @param x       x-coordinate, valid values: 1 - N
	 * @param y       y-coordinate, valid values: 1 - N
	 * @param isBound boolean true/false
	 */
	void setBoundAt(int x, int y, bool isBound);

	/**
	 * Accesor: returns boundary value at given cell.
	 *
	 * @param x       x-coordinate, valid values: 1 - N
	 * @param y       y-coordinate, valid values: 1 - N
	 * @return        boolean indicating boundary condition. 
	 */
	bool isBoundAt(int x, int y);

	/**
	 * Accessor: returns density value at a particular coordinate. Valid indicies range 
	 * from 1 to N. Indicies 0 and N+1 are buffer rows for algorithms.
	 *
	 * @param x X-coordinate, valid values: 1 - N
	 * @param y y-coordinate, valid values: 1 - N
	 * @return  Density value at coordinate.
	 */
	float getDensityAt(int x, int y);


	/**
	 * Accessor: returns vertical velocity value at a particular coordinate. 
	 * Valid indicies range from 1 to N. Indicies 0 and N+1 are buffer rows for 
	 * algorithms.
	 *
	 * @param x X-coordinate, valid values: 1 - N
	 * @param y y-coordinate, valid values: 1 - N
	 * @return  Vertical velocity value at coordinate.
	 */
	float getVertVelocityAt(int x, int y);


	/**
	 * Accessor: returns horizontal velocity value at a particular coordinate. 
	 * Valid indicies range from 1 to N. Indicies 0 and N+1 are buffer rows for 
	 * algorithms.
	 *
	 * @param x X-coordinate, valid values: 1 - N
	 * @param y y-coordinate, valid values: 1 - N
	 * @return  Horizontal  velocity value at coordinate.
	 */
	float getHorzVelocityAt(int x, int y);


	/**
	 * Runs an iteration of the simulation, updating density and velocity values.
	 * Also resets u_prev, v_prev, and dens_prev.
	 *
	 */
	void update(); 


	/**
	 * Resets all public array values to zero.
	 */
	void reset();

protected:
	float* u_;
	float* v_;
	float* u_prev_;
	float* v_prev_;
	float* dens_;
	float* dens_prev_;
	bool*  bounds_;

	int   N_;
	float dt_;
	float diff_;
	float visc_;



	/**
	 * Calculates size, including buffer cells.
	 * @return Total size of fluid simulation array, including buffer cells
	 */
	int getSize(); 



	/**
	 * Tests to see whether a coordinate is inside the valid range of fluid computation
	 * cells. (1 - N)
	 * @param x - y-coordinate
	 * @param y - y-coordinate to test
	 * @return True if the coordinate pair is within the valid range, false if not.
	 */
	bool isValidCoordinate(int x, int y);



	/** 
	 * Adds values to a matrix array, scaling the values by the timestep.
	 * @param x - reference to a float matrix array that values will be added to
	 * @param s - reference to a float matrix array representing how much value to add
	 */
	void addSource(float* x, float* s);



	/**
	 * Sets the boundaries the fluid will collide with. The horizontal component of the velocity should 
	 * be zero on the vertical walls, while the vertical component of 
	 * the velocity should be zero on the horizontal walls. 
	 *
	 * @param boundsFlag - flag value: - 
					1 - calculate collision response for horizontal velocity component (u) cells.
					2 - calculate collision response for vertical velocity component (v) cells.
	 * @param *x - pointer to a float array representing the horizontal velocity components
	 */
	void setBounds(int boundsFlag, float* x);



	/**
	 * Use Gauss-Seidel relaxation on elements in the matrix to work backwards in time 
	 * to find the or velocities densities we started with.
	 *
	 * @param b  - boundary condition flag
	 * @param x  - pointer to an array containing final solution values
	 * @param x0 - pointer to an array containing a guess of the initial solution
	 * @param a  - coefficient of relaxation per cell (dt * diffusion rate * total number of cells)
	 * @param c  - divide out least common denominator from fraction addition
	 */
	void linearSolve ( int boundsFlag, float* x, float* x0, float a, float c);



	/**
	 * Diffuse density values among surrounding cells
	 * 
	 * @param b    - boundary condition flag
	 * @param x    - pointer to an array containing final density values
	 * @param x0   - pointer to an array containing initial density values
	 */
	void diffuse ( int boundsFlag, float* x, float* x0);



	/**
	 * Move density or velocity values along a velocity field by treacting each cell center as 
	 * a particle and using a simple linear backtrace. 
	 * 
	 * @param b    - boundary condition flag
	 * @param d    - pointer to a matrix array containing final density or velocity values
	 * @param d0   - pointer to a matrix array containing initial density or velocity values
	 * @param u    - pointer to a matrix array containing horizontal velocity components
	 * @param v    - pointer to a matrix array containing vertical velocity components
	 */
	void advect (int boundsFlag, float* d, float* d0, float* u, float* v);
	


	/**
	 * Get the velocity field to conserve mass by using a Hodge decomposition.
	 *
	 * @param u    - pointer to a matrix array containing x components of velocity
	 * @param v	   - pointer to a matrix array containing y components of velocity 
	 * @param p	   - pointer to a matrix array containing projected solution
	 * @param div  - pointer to a matrix array containing divergence values
	 */
	void project (float* u, float* v, float* p, float* div);



	/**
	 * Performs density calculations per timestep 
	 *
	 * @param x	   - pointer to a matrix array of final density values
	 * @param x0   - pointer to a matrix array representing density values added this timestep
	 * @param u    - pointer to a matrix array containing x components of velocity
	 * @param v	   - pointer to a matrix array containing y components of velocity
	 */
	void computeDensityStep ( float* x, float* x0, float* u, float* v );



	/**
	 * Performs velocity calculations per timestep 
	 *
	 * @param u    - pointer to a matrix array containing x components of velocity
	 * @param v	   - pointer to a matrix array containing y components of velocity
	 * @param u0   - pointer to a matrix array containing x components of velocity to be added
	 *				 this timestep.
	 * @param u0   - pointer to a matrix array containing y components of velocity to be added
	 *				 this timestep.
	 */
	void computeVelocityStep (float* u, float* v, float* u0, float* v0);
};

