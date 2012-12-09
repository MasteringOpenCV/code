/*****************************************************************************
*   Ch9 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*   http://code.google.com/p/fluidwall/
*****************************************************************************/
/**
 * @file      FluidSolverMultiUser.cpp
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

#include "FluidSolverMultiUser.h"

#define ROW_WIDTH N_+2
#define IX(i,j) ((i)+(ROW_WIDTH)*(j))
#define FOR_EACH_CELL for (i=1 ; i<=N_ ; i++) { for (j=1 ; j<=N_ ; j++) {
#define END_FOR }}
#define SWAP(x0,x) { float* tmp=x0; x0=x; x=tmp; }
#define SWAP2D(x0,x) {float ** tmp=x0; x0=x; x=tmp;}

////// public methods
FluidSolverMultiUser::FluidSolverMultiUser(int nUsers, int N, float dt, float diff, float visc) :
	FluidSolver(N, dt, diff, visc)
{
	nUsers_ = nUsers;
	int size = getSize();

	userDensity_      = new float*[nUsers_];
	userDensity_prev_ = new float*[nUsers_];


	for(int i = 0; i < nUsers_; i++) {
		userDensity_[i]      = new float[size];
		userDensity_prev_[i] = new float[size];
	}

	reset();
}



FluidSolverMultiUser::~FluidSolverMultiUser(void)
{
}



void FluidSolverMultiUser::addDensityAt(int userID, int x, int y, float value)
{
	if(isValidCoordinate(x, y))
		userDensity_prev_[userID][IX(x,y)] += value * dt_;
}



float FluidSolverMultiUser::getDensityAt(int userID, int x, int y)
{
	return userDensity_[userID][IX(x,y)];
}



void FluidSolverMultiUser::update()
{
	for(int i = 0; i < nUsers_; i++)
		computeDensityStep(userDensity_[i], userDensity_prev_[i], u_, v_);
	computeVelocityStep(u_, v_, u_prev_, v_prev_);

	//reset u_prev_, v_prev_, and dens_prev
	for(int i = 0; i < getSize(); i++) { 
		u_prev_[i] = v_prev_[i] = 0.0f;
	}

	resetUserDensities(userDensity_prev_);	
}

void FluidSolverMultiUser::reset()
{
	for(int i = 0 ; i < getSize(); i++) {
		u_[i] = v_[i] = u_prev_[i] = v_prev_[i] = 0.0f;
		bounds_[i] = false;
	}

	resetUserDensities(userDensity_);
	resetUserDensities(userDensity_prev_);
}


////// protected methods
void FluidSolverMultiUser::resetUserDensities(float** userDensity)
{
	for(int i = 0; i < nUsers_; i++)
		for(int j = 0; j < getSize(); j++) {
			if(i == 0) //the user "no user" (0) has full weight
				userDensity[i][j] = 1.0f;
			else
				userDensity[i][j] = 0.0f;
		}
}