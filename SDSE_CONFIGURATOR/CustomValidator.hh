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


#ifndef __CUSTOM_VALIDATOR_HH_DEF__
#define __CUSTOM_VALIDATOR_HH_DEF__

#include <QValidator>

// ----------------------------------------------------------------------------
// Allow to filter entries in QLineEdit to make sure the text
// entered is between the limits and has a correct syntax.
// ----------------------------------------------------------------------------
class CustomValidator : public QDoubleValidator
{
public:

    CustomValidator(double bottom, double top, int decimals, QObject * parent=NULL) :
        QDoubleValidator(bottom, top, decimals, parent)
    {
    }

    QValidator::State validate(QString &s, int &i) const
    {
        if ( s.isEmpty() || ((s == "-")&&(bottom()<0)) ) {
            return QValidator::Intermediate;
        }

        QChar decimalPoint = locale().decimalPoint();

        if(s.indexOf(decimalPoint) != -1) {
            int charsAfterPoint = s.length() - s.indexOf(decimalPoint) - 1;
            if (charsAfterPoint == 0){
                return QValidator::Intermediate;
            } else if (charsAfterPoint > decimals()) {
                return QValidator::Invalid;
            }
        }

        bool ok;
        double d = locale().toDouble(s, &ok);

        if (bottom() <= 0){
            if (ok && d >= bottom() && d <= top()) {
                return QValidator::Acceptable;
            } else {
                return QValidator::Invalid;
            }
        } else {
            if (ok && d >= bottom() && d <= top()) {
                return QValidator::Acceptable;
            } else if (ok && d > 0 && d <= top()){
                return QValidator::Intermediate;
            } else {
                return QValidator::Invalid;
            }
        }
        (void)i; // Does nothing but eliminate "unused variable" warning
    }
};

#endif // __CUSTOM_VALIDATOR_HH_DEF__
