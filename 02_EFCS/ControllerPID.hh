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

#ifndef __CONTROLLER_PID_HH__
#define __CONTROLLER_PID_HH__

class ControllerPID
{
	public:
		ControllerPID();
		~ControllerPID();
		void setParameters( double Kp
		                  , double Ts
		                  , double Ti
		                  , double Td
		                  , double Alpha
		                  , double Beta
		                  , double Gamma
		                  , double Min
		                  , double Max
		                  );
		double calculatePID(double Input, double Ref);

	private: 
		
		double _Kp; 
		double _Ep, _EpL; 
		double _Ed, _EdL; 
		double _En;
		double _Edf, _EdfL, _EdfLL;
		double _DetlaUn;
		double _Output; 
		double _OutputL; 
		double _Beta, _Alpha, _Gamma;
		double _Ts, _Ti, _Td, _Tf;
		double _Max; // Max Saturation
		double _Min; // Min Saturation		
};

#endif
