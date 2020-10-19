// ----------------------------------------------------------------------------
// OpenSDSE - HLA Compliant Distributed Aircraft Simulation
// Copyright (C) 2017  ISAE
//
// This program is free software ; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation ; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY ; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program ; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
//
// ----------------------------------------------------------------------------

#ifndef __JOYSTICK_HH_DEF__
#define __JOYSTICK_HH_DEF__

#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#ifdef __linux__
#include <linux/joystick.h> // Linux-only API
#endif


const int MAX_AXIS   = 7;
const int MAX_BUTTON = 24;
const int MAX_INPUTS = MAX_AXIS+MAX_BUTTON;
const double MAX_INT = 32767;

using namespace std;

// ----------------------------------------------------------------------------
// Handle all the interactions between the Joystick and the simulation.
// It was designed to get inputs from two joysticks (J0&J1)
// The current implementation works on Linux by using the joystick.h API.
// TO DO: Implement a cross platform solution for handling Joystick events.
// ----------------------------------------------------------------------------
class Joystick
{
	public:

		Joystick();

		virtual ~Joystick();
		double* getInputs();

		void setInputs(int index,double value);
		void setFailureInputs(int index,int value);

		// Failure mode
		int _FailureInputs[2*MAX_INPUTS];

	private:

		unsigned char mJ0AxisCount;
		unsigned char mJ1AxisCount;
		unsigned char mJ0ButtonCount;
		unsigned char mJ1ButtonCount;
		int mJ0Fd;
		int mJ1Fd;

		int mJ0Axis[MAX_AXIS];
		int mJ1Axis[MAX_AXIS];
		int mJ0Button[MAX_BUTTON];
		int mJ1Button[MAX_BUTTON];

		// The inputs are built like this: [J0 axis,J0 buttons,J1 axis,J1 buttons]
		double mInputs[2*MAX_INPUTS];

		int mJ0Result;
		int mJ1Result;

		#ifdef __linux__
		js_event mEvent; // is part of the Linux-only joystick API
		#endif
};

#endif // __JOYSTICK_HH_DEF__
