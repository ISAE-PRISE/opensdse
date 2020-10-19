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

#include <iostream>
#include "Joystick.hh"
#include "Common.hh"

using namespace std;


// ----------------------------------------------------------------------------
//
Joystick::Joystick()
{
	// Joystick is disabled regardless of JOYSTICK_ENABLED for platforms
	// that don't support the Linux-only joystick API
	#ifdef __linux__
	if (JOYSTICK_ENABLED)
	{

		mJ0Fd  = open("/dev/input/js0", O_RDONLY);
		mJ1Fd  = open("/dev/input/js1", O_RDONLY);

		#ifdef DEBUG_JOYSTICK
		if(mJ0Fd != -1)cout<<"Joystick 0 detected"<<endl;
		if(mJ1Fd != -1)cout<<"Joystick 1 detected"<<endl;
		#endif

		ioctl(mJ0Fd, JSIOCGAXES, &mJ0AxisCount);
		ioctl(mJ1Fd, JSIOCGAXES, &mJ1AxisCount);
		ioctl(mJ0Fd, JSIOCGBUTTONS, &mJ0ButtonCount);
		ioctl(mJ1Fd, JSIOCGBUTTONS, &mJ1ButtonCount);

		fcntl(mJ0Fd, F_SETFL, O_NONBLOCK);
		fcntl(mJ1Fd, F_SETFL, O_NONBLOCK);

		for (int i=0;i<mJ0AxisCount;i++) mJ0Axis[i]=0;
		for (int i=0;i<mJ1AxisCount;i++) mJ1Axis[i]=0;
		for (int i=0;i<mJ0ButtonCount;i++) mJ0Button[i]=0;
		for (int i=0;i<mJ1ButtonCount;i++) mJ1Button[i]=0;

		// Specific Initialization to ensure 0 position throttle at beginning
		//mYokeAxis[2] = (int) MAX_INT;
	  }

	#endif // __linux__

	for(int i=0;i<2*MAX_INPUTS;i++)
	{
		mInputs[i] = 0;
	}
	for(int i=0;i<2*MAX_INPUTS;i++)
	{
		_FailureInputs[i]=0;
	}
	mJ0Result = 0;
	mJ1Result = 0;
}

// ----------------------------------------------------------------------------
//
Joystick::~Joystick()
{

}

// ----------------------------------------------------------------------------
// getInputs does the acquisition of inputs from both joysticks
// Returns the inputs of a joystick if compiled on Linux and JOYSTICK_ENABLED flag is set to true,
// otherwise it returns an array of 0.
double* Joystick::getInputs()
{
	// Joystick is disabled regardless of JOYSTICK_ENABLED for platforms
	// that don't support the Linux-only joystick API
	#ifdef __linux__
	if (JOYSTICK_ENABLED) 
	{
		// J0 System Acquisition
		if(mJ0Fd!=-1) mJ0Result = read(mJ0Fd, &mEvent, sizeof(mEvent));
		if (mJ0Result > 0)
		{
			switch (mEvent.type)
			{
				case JS_EVENT_INIT:
				case JS_EVENT_INIT | JS_EVENT_AXIS:
					mJ0Axis[mEvent.number] = mEvent.value;
					break;

					case JS_EVENT_INIT | JS_EVENT_BUTTON:
					mJ0Button[mEvent.number] = mEvent.value;
					break;

					case JS_EVENT_AXIS:
					mJ0Axis[mEvent.number] = mEvent.value;
					break;

					case JS_EVENT_BUTTON:
					mJ0Button[mEvent.number] = mEvent.value;
					break;

					default:
				cout << "Other j0 event ?" << endl;
					break;
			}
		}
		else
		{
			usleep(1);
		}

		// J1 System Acquisition
		if(mJ1Fd!=-1) mJ1Result = read(mJ1Fd, &mEvent, sizeof(mEvent));
		if (mJ1Result > 0)
		{
			switch (mEvent.type)
			{
				case JS_EVENT_INIT:
				case JS_EVENT_INIT | JS_EVENT_AXIS:
					mJ1Axis[mEvent.number] = mEvent.value;
					break;

					case JS_EVENT_INIT | JS_EVENT_BUTTON:
					mJ1Button[mEvent.number] = mEvent.value;
					break;

					case JS_EVENT_AXIS:
					mJ1Axis[mEvent.number] = mEvent.value;
					break;

					case JS_EVENT_BUTTON:
					mJ1Button[mEvent.number] = mEvent.value;
					break;

					default:
				cout << "Other j1 event ?" << endl;
					break;
			}
		}
		else
		{
			usleep(1);
		}

		for(int i=0;i<MAX_AXIS;i++)
		{
			if (_FailureInputs[i]==0) {mInputs[i] = (static_cast<double>(mJ0Axis[i]))/( MAX_INT );}
		}
		for(int i=0;i<MAX_BUTTON;i++)
		{
			if (_FailureInputs[i+MAX_AXIS]==0) {mInputs[i+MAX_AXIS] = (static_cast<double>(mJ0Button[i]));}
		}
		for(int i=0;i<MAX_AXIS;i++)
		{
			if (_FailureInputs[i+MAX_INPUTS]==0) {mInputs[i+MAX_INPUTS] = (static_cast<double>(mJ1Axis[i]))/( MAX_INT );}
		}
		for(int i=0;i<MAX_BUTTON;i++)
		{
			if (_FailureInputs[i+MAX_INPUTS+MAX_AXIS]==0) {mInputs[i+MAX_INPUTS+MAX_AXIS] = (static_cast<double>(mJ1Button[i]));}
		}
	}
	#endif // __linux__
	return mInputs;
}

// ----------------------------------------------------------------------------
//
void Joystick::setInputs(int index,double value)
{
    if (index>=0 && index<2*MAX_INPUTS)
    {
        mInputs[index] = value;
	}
}

// ----------------------------------------------------------------------------
//
void Joystick::setFailureInputs(int index,int value)
{
    if (index>=0 && index<2*MAX_INPUTS)
    {
        _FailureInputs[index] = value;
	}
}
