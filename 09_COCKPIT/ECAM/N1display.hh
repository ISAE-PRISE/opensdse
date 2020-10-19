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

#ifndef __N1_DISPLAY_HH_DEF__
#define __N1_DISPLAY_HH_DEF__

#include <QGraphicsView>
#include <QtSvg/QGraphicsSvgItem>
#include <QtSvg/QSvgGenerator>

// ----------------------------------------------------------------------------
// Provide a GUI for arrow indicators
// ----------------------------------------------------------------------------
class N1display : public QGraphicsView 
{

	public:

		N1display( QWidget *parent = 0 );
		~N1display( void );
		void    init( void );
		void    set( double power );
		enum RendererType { Native, OpenGL, Image };

	private:

		void    clean( void );
		void    update_view( void );
		void    switch_color_mode( void );

		double  actual_pwr;         // current power in %
		double  previous_pwr;       // previous power in %

		double red_step;            // red/green step in %

		RendererType m_renderer;

		QGraphicsSvgItem *item_background;
		QGraphicsSvgItem *item_scale;
		QGraphicsSvgItem *item_window_green;
		QGraphicsSvgItem *item_window_red;
		QGraphicsTextItem *item_window_text;
		QGraphicsSvgItem *item_arrow_green;
		QGraphicsSvgItem *item_arrow_red;
		bool color_mode; // 0=red, 1=green
    
};

#endif //  __N1_DISPLAY_HH_DEF__

