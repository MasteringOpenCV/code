/*****************************************************************************
*   Ch9 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*   http://code.google.com/p/fluidwall/
*****************************************************************************/
/**
 * @file      FluidSolverMultiUser.h
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
#include "fluidsolver.h"

/**
 * Extends the 2D FluidSolver to support simulation of densities that represent various 
 * users in the Fluid Wall simulation
 *
 */
class FluidSolverMultiUser :
	public FluidSolver
{
public:
	/**
	 * Parameter constructor
	 * @param nUsers Number of users that the solver will calculate.
	 * @param N      Width (and height) of the square fluid simulation grid
	 * @param dt     Timestep size
	 * @param diff   Diffusion coefficient
	 * @param visc   Viscosity coefficient
	 */
	FluidSolverMultiUser(int nUsers, int N, float dt, float diff, float visc);
	~FluidSolverMultiUser(void);

	/**
	 * Adds density values at a particular coordinate. 
	 *
	 * The system is designed to add values from multiple other sources using this method, 
	 * and then commit them to the simulation using update(). Valid indicies range from 
	 * 1 to N. Indicies 0 and N+1 are buffer rows for algorithms.
	 *
	 * @param userID User number to add density to
	 * @param x     x-coordinate, valid values: 1 - N
	 * @param y     y-coordinate, valid values: 1 - N
	 * @param value  Density value to be added
	 */
	void addDensityAt(int userID, int x, int y, float value);

	/**
	 * Accessor: returns density value for given user at given coordinate. Valid indicies range 
	 * from 1 to N. Indicies 0 and N+1 are buffer rows for algorithms.
	 *
	 * @param userID  User ID.
	 * @param x		  X-coordinate, valid values: 1 - N
	 * @param y		  y-coordinate, valid values: 1 - N
	 * @return		  Density value at coordinate.
	 */
	float getDensityAt(int userID, int x, int y);

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
	int     nUsers_;
	float** userDensity_;
	float** userDensity_prev_;

	/**
	 * Resets values in userDensity so that user 0 
	 * has 1.0 density.
	 */
	void resetUserDensities(float** userDensity);
	//normalize


};

