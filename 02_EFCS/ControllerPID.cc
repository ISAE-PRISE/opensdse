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

#include "ControllerPID.hh"

// ----------------------------------------------------------------------------
// Constructor
ControllerPID::ControllerPID()
{
	_Kp = 0.0; 		
	_Ep = 0.0;
	_EpL= 0.0;
	_Ed = 0.0;
	_EdL= 0.0;
	_En = 0.0;
	_Edf = 0.0; 
	_EdfL = 0.0;
	_EdfLL = 0.0;	
	_Output = 0.0; 
	_OutputL = 0.0;	
	_Beta = 1.0; 
	_Alpha = 0.1; 
	_Gamma = 0.0;
	_Ts = 0.0; 
	_Ti = 0.0; 
	_Td = 0.0;
	_Tf = 0.0;
	_DetlaUn = 0.0;
	_Max 			= 0.0; 
	_Min 			= 0.0; 
}

// ----------------------------------------------------------------------------
// Destructor
ControllerPID::~ControllerPID()
{
	// Nothing to do here
}

// ----------------------------------------------------------------------------
// function to set correct value for Controller
void ControllerPID::setParameters( double Kp
		                         , double Ts
		                         , double Ti
		                         , double Td
		                         , double Alpha
		                         , double Beta
		                         , double Gamma
		                         , double Min
		                         , double Max
		                         )
{
	_Kp = Kp; 		
	_Ts = Ts; 
	_Ti = Ti; 
	_Td = Td;
	_Alpha = Alpha; 
	_Beta = Beta; 
	_Gamma = Gamma;
	_Tf = _Alpha * _Ts;
	_Min = Min;
	_Max = Max;
}

// ----------------------------------------------------------------------------
// Function: Calculate the PID controller output value for a given entry
double ControllerPID::calculatePID(double Input, double Ref)
{
	_Ep = _Beta * Ref - Input;
	_En = Ref - Input;
	_Ed = _Gamma * Ref - Input;
	_Edf = _EdfL / ((_Ts/_Tf) + 1) + _Ed * ((_Ts/_Tf) / ((_Ts/_Tf) + 1));
	_DetlaUn = _Kp * ( (_Ep - _EpL) + ((_Ts/_Ti) * _En) + ((_Td/_Ts) * (_Edf - 2.0*_EdfL + _EdfLL)));
	_Output = _OutputL + _DetlaUn;
	
	// Saturation
	if     (_Output > _Max)
	{
		_Output = _Max;
	}
    else if(_Output < _Min) 
    {
		_Output = _Min;
	}
	
	_EpL = _Ep;
	_EdL = _Ed;
	_EdfL = _Edf;
	_EdfLL = _EdfL;
	_OutputL = _Output;
	
	return _Output;
}
