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

#ifndef __ECAM_WINDOW_HH_DEF__
#define __ECAM_WINDOW_HH_DEF__

#include <QWidget>

namespace Ui 
{
	class ECAMwindow;
}

// ----------------------------------------------------------------------------
// Provide a GUI for the Electronic Centralised Aircraft Monitor
// ----------------------------------------------------------------------------
class ECAMwindow : public QWidget
{
	Q_OBJECT

	public:
		explicit ECAMwindow(QWidget *parent = 0);
		~ECAMwindow();

		void set_SDSE_left_engine_thr(double left_thr) {SDSE_left_engine_thr = left_thr; }
		void set_SDSE_right_engine_thr(double right_thr) {SDSE_right_engine_thr = right_thr; }
		void set_SDSE_mach(double mch) {SDSE_mach = mch; }
		void set_SDSE_pressure(double pres) {SDSE_pressure = pres; }

		void consider_new_data();

	private:
		Ui::ECAMwindow *ui;

		double SDSE_left_engine_thr;
		double SDSE_right_engine_thr;
		double SDSE_mach;
		double SDSE_pressure;

		double max_thrust_engine;
		double pressureSL;
		double machratio;
		double pressureratio;

		double N1powerleft;
		double N1powerright;

};

#endif // __ECAM_WINDOW_HH_DEF__
