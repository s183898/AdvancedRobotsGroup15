/***************************************************************************
 *   Copyright (C) 2005 by Christian Andersen and DTU                             *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef UFUNC_MRCOBST_H
#define UFUNC_MRCOBST_H

#include <cstdlib>

#include <ulms4/ufunclaserbase.h>
#include <urob4/uresposehist.h>
#include <vector>


////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
 * Laserscanner function to demonstrate
 * simple laser scanner data handling and analysis
 * @author Christian Andersen
*/
class boxDetection : public UFuncLaserBase
{
public:
  /**
  Constructor */
  boxDetection()
  { // set the command (or commands) handled by this plugin
    setCommand("detectBox", "defineObject", "obstacle detect for MRC (Compiled " __DATE__ " " __TIME__ ")");
    createBaseVar();
  }
  virtual bool setResource(UResBase * resource, bool remove);
  /**
  Handle incomming command
  (intended for command separation)
  Must return true if the function is handled -
  otherwise the client will get a failed - reply */

  virtual void linesDetection (std::vector <double>lines_x, std::vector <double>lines_y);
  virtual double segmentSize(std::vector <double>line_x, std::vector <double>line_y);
  virtual std::vector<double> findCenter(std::vector <double>line_x, std::vector <double>line_y, double angle, double h);
  virtual void boxParameters (std::vector <double>line1_x, std::vector <double>line1_y, std::vector <double>line2_x, std::vector <double>lines_y, bool splitted);
  virtual std::vector<double> lsqline(const std::vector<double>& x, const std::vector<double>& y);
  virtual std::vector<double> transform(const std::vector<double>& pose, double &x, double &y);
  virtual std::vector<double> transformL2W(std::vector<double> &pose);
  virtual bool handleCommand(UServerInMsg * msg, void * extra);


  protected:
    void createBaseVar();
    UVariable *var_zone;
    UResPoseHist * poseHist;
};



#endif
