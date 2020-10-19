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


#ifndef __COCKPIT_FEDERATE_THREAD_HLA13_HH__
#define __COCKPIT_FEDERATE_THREAD_HLA13_HH__

#include "CockpitMainWindow.hh"
#include <QThread>


class CockpitFederateThreadHla13: public QThread
{
public:
    void run();
    static CockpitMainWindow *window;
    CockpitFederateThreadHla13(CockpitMainWindow*);
    
Q_OBJECT
    
signals:
	void gps_update_signal();
	void pfd_update_signal();
	void ecam_update_signal();
};

#endif 
