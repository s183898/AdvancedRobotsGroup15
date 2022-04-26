/***************************************************************************
 *   Copyright (C) 2005 by Christian Andersen and DTU                      *
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
#include "boxdetection.h"
#include "math.h"
#include "vector"

#define PI 3.14159265

using namespace std;

#ifdef LIBRARY_OPEN_NEEDED


/**
 * This function is needed by the server to create a version of this plugin */
UFunctionBase * createFunc()
{ // create an object of this type
  /** replace 'boxDetection' with your class name */
  return new boxDetection();
}
#endif

bool boxDetection::setResource(UResBase * resource, bool remove) { // load resource as provided by the server (or other plugins)
	bool result = true;

	if (resource->isA(UResPoseHist::getOdoPoseID())) { // pointer to server the resource that this plugin can provide too
		// but as there might be more plugins that can provide the same resource
		// use the provided
		if (remove)
			// the resource is unloaded, so reference must be removed
			poseHist = NULL;
		else if (poseHist != (UResPoseHist *) resource)
			// resource is new or is moved, save the new reference
			poseHist = (UResPoseHist *) resource;
		else
			// reference is not used
			result = false;
	}

	// other resource types may be needed by base function.
	result = UFunctionBase::setResource(resource, remove);
	return result;
}


// My variables
int vectorBreakPoint;
bool splitLine;
std::vector <double> line1_x, line1_y;
std::vector <double> line2_x, line2_y;

std::vector<double> AB_params(2), CD_params(2); // Each of them carries the angle and the distance respectively;
std::vector<double> center_L(2); // first output of this plugin
double orientation; // second output of this plugin

void boxDetection::linesDetection (vector <double>lines_x, vector <double>lines_y){

  sendText("linesDetection_begin\n");

  vectorBreakPoint = -1;
  for (std::vector<float>::size_type i = 1; i < (lines_x.size() -2); i++){
  sendText("\n lines splitting begin, x coordinates");

		double first_element = lines_x[i - 1];
		double second_element = lines_x[i];
		double third_element = lines_x[i + 1];

		bool bool1 = first_element > second_element;
		bool bool2 = second_element > third_element;
		bool bool3 = first_element < second_element;
		bool bool4 = second_element < third_element;

		if ((bool1 == true && bool2 == true) || (bool3 == true && bool4 == true)){
		    if (i == lines_x.size() - 2){
		        cout << "\n Only one line detected by x coordinates\n";
		        vectorBreakPoint = -2;
		        splitLine = false;
		    }
		} else {
		    cout << "\n The vector breaks here:\n" << i << "\n";
		    vectorBreakPoint = i;
		    splitLine = true;
		    break;
		}
    }
    sendText("\n x lines splitting ended");

    if (vectorBreakPoint == -2 && splitLine == false){
    sendText("\n lines splitting begin, y coordinates");

        for (std::vector<float>::size_type i = 1; i < lines_y.size()-2; i++){

		double first_element = lines_y[i - 1];
		double second_element = lines_y[i];
		double third_element = lines_y[i + 1];

		bool bool1 = first_element > second_element;
		bool bool2 = second_element > third_element;
		bool bool3 = first_element < second_element;
		bool bool4 = second_element < third_element;

		if ((bool1 == true && bool2 == true) || (bool3 == true && bool4 == true)){
		    if (i == lines_y.size() - 2){
		        cout << "\nOnly one line detected by y coordinates\n";
		        vectorBreakPoint = -3;
		        splitLine = false;
		    }
		} else {
		    cout << "\nThe vector breaks here:\n" << i << "\n";
		    vectorBreakPoint = i;
		    splitLine = true;
		    break;
		}
    }
    }
   sendText("\n y lines splitting ended");

    if (vectorBreakPoint > 0 && splitLine == true){
        line1_x.assign (lines_x.begin (),lines_x.begin () + vectorBreakPoint + 1);
        line1_y.assign (lines_y.begin (),lines_y.begin () + vectorBreakPoint + 1);
        line2_x.assign (lines_x.begin () + vectorBreakPoint + 1,lines_x.end ());
        line2_y.assign (lines_y.begin () + vectorBreakPoint + 1,lines_y.end ());
    }
    if (vectorBreakPoint == -3 && splitLine == false){
        line1_x = lines_x;
        line1_y = lines_y;
    }
}


double boxDetection::segmentSize(vector <double>line_x, vector <double>line_y){
  double Ax, Ay, Bx, By;
  Ax = *line_x.begin();
  Ay = *line_y.begin();
  Bx = *line_x.rbegin();
  By = *line_y.rbegin();

  double a = pow((Ax - Bx),2);
  double b = pow((Ay - By),2);

  double size = sqrt(a+b);

  cout << "\n The side has lenght: " << size << "\n";
  return size;
}

vector<double> boxDetection::findCenter(vector <double>line_x, vector <double>line_y, double angle, double h){
  double Ax, Ay, Bx, By;
  Ax = *line_x.begin();
  Ay = *line_y.begin();
  Bx = *line_x.rbegin();
  By = *line_y.rbegin();

  // Coordinates of the middle point of the side AB
  double a = (Ax + Bx)/2;
  double b = (Ay + By)/2;

  // finding the coordinates of the center:
  double center_x = h*cos(angle)+a;
  double center_y = h*sin(angle)+b;

  cout<< "\nThe center of the rectangle in laser coordinates frame is:\n x:" << center_x << "\n y:" << center_y <<"\n";

  vector<double> center;
  center.reserve(2);
  center.push_back(center_x);
  center.push_back(center_y);

  return center;
}

void boxDetection::boxParameters (vector <double>line1_x, vector <double>line1_y, vector <double>line2_x, vector <double>lines_y, bool splitted){
  double AB, CD;
  if(splitted == false){
    // We only have detected one line.
    AB = segmentSize(line1_x, line1_y);
    cout << "\n AB is: " << AB << "\n";

    if(AB>0.275){
        AB_params = lsqline(line1_x,line1_y);
        orientation = AB_params[0];

        cout << "AB params: " << AB_params[0] << ", " << AB_params[1] << "\n";

        center_L = findCenter(line1_x,line1_y,AB_params[0],0.10);

    } else if(AB<0.20){
        AB_params = lsqline(line1_x,line1_y);
        // since we have detected only the short side we need to evaluate manually the orientation of the longer one
        orientation = AB_params[0]+PI/2;

        cout << "AB params: " << AB_params[0] << ", " << AB_params[1] << "\n";

        center_L = findCenter(line1_x,line1_y,AB_params[0],0.15);
    }


  } else if (splitted == true){
    // We have detected two lines.
    vector <double> AB_x, AB_y;
    if(line1_x.size()>4){
        AB = segmentSize(line1_x, line1_y);
        AB_params = lsqline(line1_x, line1_y);
        cout << "\n AB is: " << AB << "\n";

        if(line2_x.size()>4){
            CD = segmentSize(line2_x,line2_y);
            CD_params = lsqline(line2_x,line2_y);
            cout << "\n CD is: " << CD << "\n";

        } else {
            CD = 0;
            AB_x = line1_x;
            AB_y = line1_y;
            cout << "Too few measurement to take CD into consideration for further measurements.";
        }
    } else {
        CD = 0;
        AB_x = line2_x;
        AB_y = line2_y;
        AB = segmentSize(line2_x, line2_y);
        AB_params = lsqline(line2_x, line2_y);
        cout << "Too few measurement to take line 1 into consideration.\n Line 2 is now AB.\n";
        cout << "\n AB is: " << AB << "\n";
    }

    if (CD != 0){
        // case in which both line are well defined
        if (AB>CD){
            // AB is the longer side so we need its orientation as output of the plugin
            orientation = AB_params[0];

            // Now we need to find the center. This will be found using the longer side as a reference
            center_L = findCenter(line1_x,line1_y,AB_params[0],0.10);
        } else {
            // CD is the longer side so we need its orientation as output of the plugin
            orientation = CD_params[0];

            // Now we need to find the center. This will be found using the longer side as a reference
            center_L = findCenter(line2_x,line2_y,AB_params[0],0.10);
        }

    } else if (CD == 0) {
        // case in which only one line is well defined
        if (AB>0.275){
            // AB is the longer side so we need its orientation as output of the plugin
            orientation = AB_params[0];

            // Now we need to find the center. This will be found using the longer side as a reference
            center_L = findCenter(AB_x,AB_y,AB_params[0],0.10);
        } else {
            // AB is the shorter side so we need evaluate the orientation of the other side
            orientation = AB_params[0]+PI/2;

            // Now we need to find the center. This will be found using the longer side as a reference
            center_L = findCenter(AB_x,AB_y,AB_params[0],0.15);
        }
    }

  }
}

vector<double> boxDetection::lsqline(const vector<double>& x, const vector<double>& y)
{
  /*
  * This function implements the core algorithm to do least squares approximation of
  * (x,y) data points to fit a line.
  *
  * INPUT:  vector<double> x: X values of a laser scan
  *         vector<double> y: Y values of a laser scan
  *
  * OUTPUT:
  *         vector<double>: 1-D vector containing alpha,r parameters of the line fit
  * */

  int n = x.size();
  double xmean, ymean, sumx, sumy, sumx2, sumy2, sumxy;

  for (int j = 0; j < n; j++)
  {
    sumx += x[j];
    sumy += y[j];
  }

  xmean = sumx / (double)n;
  ymean = sumy / (double)n;

  sumx2 = 0;
  sumy2 = 0;
  sumxy = 0;
  for (int i = 0; i < n; i++)
  {
    sumx2 += x[i] * x[i];
    sumy2 += y[i] * y[i];
    sumxy += x[i] * y[i];
  }

  double a = 0.5 * atan2((2 * sumx * sumy - 2 * (double)n * sumxy), pow(sumx, 2) - pow(sumy, 2) - (double)n * sumx2 + (double)n * sumy2);
  double r = xmean * cos(a) + ymean * sin(a);

  if (r < 0)
  {
    r = abs(r);
    if (a < 0)
    {
      a += PI;
    }
    else
    {
      a -= PI;
    }
  }

  // make sure we are in ]-pi;pi]
  a = atan2(sin(a),cos(a));

  vector<double> line;
  line.reserve(2);
  line.push_back(a);
  line.push_back(r);

  return line;
}


vector<double> boxDetection::transform(const vector<double>& pose, double &x, double &y)
{
  /*
  * This function converts a set of (x,y) points from the laser frame to the
  * world frame using the pose of the laser scanner in the world frame.
  *
  *
  * INPUT:  vector<double> pose: (x,y,th) pose of the laser scanner in the world frame
  *         double &x: address of X location of a point in the laser frame
  *         double &y: address of Y location of a point in the laser frame
  *
  * */

  double th_lw = pose[2];

  double tempx = cos(th_lw)*x - sin(th_lw)*y + pose[0];
  double tempy = sin(th_lw)*x + cos(th_lw)*y + pose[1];

  vector<double> P_w;
  P_w.reserve(2);
  P_w.push_back(tempx);
  P_w.push_back(tempy);

  return P_w;
}

vector<double> boxDetection::transformL2W(vector<double> &pose_w)
{
  /*
  * This function finds the pose of the laser scanner in the world frame using the
  * pose of the robot in the world frame.
  * The laser scanner is mounted on the robot 26 cm from the centre.
  *
  * INPUT:  vector<double> pose: address of (x,y,th) pose of the robot in the world frame to be transformed to laser pose in world
  *
  * */
  vector<double> pose;
  pose.reserve(2);

  pose[0] = cos(pose_w[2]) * 0.26 + pose_w[0];
  pose[1] = sin(pose_w[2]) * 0.26 + pose_w[1];

  return pose;
}
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
// #define SMRWIDTH 0.4
bool boxDetection::handleCommand(UServerInMsg * msg, void * extra)
{  // handle a plugin command
  const int MRL = 500;
  char reply[MRL];
  bool ask4help;
  bool detectBox = false;
  bool determineObject = false;
  const int MVL = 30;
  char val[MRL];
  char value[MVL];
  ULaserData * data;
  double robotwidth;

  //
  double xr = 0, yr = 0, thr = 0;
  //
  int i,j,imax;
  double r,delta;
  double minRange; // min range in meter

  // double minAngle = 0.0; // degrees
  // double d,robotwidth;
  double zone[9];
  // check for parameters - one parameter is tested for - 'help'
  ask4help = msg->tag.getAttValue("help", value, MVL);
  detectBox = msg->tag.getAttValue("detect", value, MVL);
  determineObject = msg->tag.getAttValue("determine", value, MVL);

  UPose poseAtScan = poseHist->getPoseAtTime(data->getScanTime());

  if (msg->tag.getAttValue("x", val, MVL))
  {
    xr = strtod(val, NULL);
  }
  if (msg->tag.getAttValue("y", val, MVL))
  {
    yr = strtod(val, NULL);
  }
  if (msg->tag.getAttValue("th", val, MVL))
  {
    thr = strtod(val, NULL);
  }

  if (ask4help)
  { // create the reply in XML-like (html - like) format
    sendHelpStart(msg, "zoneobst");
    sendText("--- available zoneobst options\n");
    sendText("help            This message\n");
    sendText("fake=F          Fake some data 1=random, 2-4 a fake corridor\n");
    sendText("device=N        Laser device to use (see: SCANGET help)\n");
    sendText("see also: SCANGET and SCANSET\n");
    sendHelpDone();
  }
  else if (detectBox) //------------------------------------------ MY CODE HERE ------------------------
  {
    // Get laser data
    data = getScan(msg, (ULaserData *)extra);
    cout << "\n Pose at scan is:\n x:" << poseAtScan[0] << "\n y:" << poseAtScan[1] << "\n theta:" << poseAtScan[2] << "\n";

    // check if data is valid
    if (data->isValid())
    {
      // create vector with robot pose (x,y,th) in world given by the caller
      vector<double> poseW, pose;
      poseW.reserve(3);
      poseW.push_back(xr);
      poseW.push_back(yr);
      poseW.push_back(thr);
      // find the pose of the laser scanner in world
      pose = transformL2W(poseW);

      // vectors to store x,y data (point in the laser frame)
      vector<double> x;
      vector<double> y;

      // loop over each data point and only store good values that fall within the "green area"
      for (int i = 0; i < data->getRangeCnt(); i++)
      {
        double range = data->getRangeMeter(i);
        double angle = data->getAngleRad(i);

        // x,y data in the laser frame
        double xx = cos(angle) * range;
        double yy = sin(angle) * range;

        // transform to world frame for test
        vector<double> P_w = transform(pose, xx, yy);
        if (P_w[0] >= 0.95 && P_w[0] <= 3.05 && P_w[1] >= 0.95 && P_w[1] <= 2.05 && range > 0.03)
        {
            // x,y data in laser frame
            x.push_back(cos(angle)*range);
            y.push_back(sin(angle)*range);
        }
      }

      // Let's detect the line(s). We can either end up with only one line or two lines detected (supposedly with a good amount of measurements).
      // Another thing that can happen is that we end up with two lines, but one of them doesn't have a sufficient amount of measurement to hallow
      // us to detect the center/size of the side with sufficient confidence. We will fix this minimum amount to be 5.

      // First let's use the function linesDetection to find the two lines:
	if (x.size() == 0){
		sendText("No box found inside the search area!");
 	} else {
		sendText("\n Starting linesDetection:\n");
 		linesDetection(x,y);
		sendText("\n Starting boxParameters:");
    cout << "\n The pose of the Laserscanner in world coordinates framse is: \n x: " << pose[0] << "\n y: "  << pose[1] << "\n theta: "  << pose[2] << "\n";
    boxParameters(line1_x,line1_y,line2_x,line2_y, splitLine);
    vector<double> center_w = transform(pose,center_L[0],center_L[1]);
    cout << "\n The center of the rectangle in wordl coordinates framse is: \n x: " << center_w[0] << "\n y: "  << center_w[1] << "\n";
    cout << "\nThe orientation of the longer side of the rectangle is: "<< orientation;
	}
    }
  }
  else // ----------------------------------------------------------------------------- TEACHER'S CODE HERE -------------
  { // do some action and send a reply
    data = getScan(msg, (ULaserData*)extra); // Get data
    //
    if (data->isValid())
    {
    // check if a attribute (parameter) with name width exists and if so get its value
       bool gotwidth = msg->tag.getAttValue("width", value, MVL);
       if (gotwidth) {
        	robotwidth=strtod(value, NULL);
       }
       else {
        	robotwidth=0.26;
      }
      UPose poseAtScan = poseHist->getPoseAtTime(data->getScanTime());
      // Gets the odometry pose at the time when the laserscan was taken, poseAtScan.x poseAtScan.y poseAtScan.a (x,y,angle)
      // make analysis for closest measurement
      minRange = 1000; // long range in meters
      imax=data->getRangeCnt();
      delta=imax/9.0;
      for (j=0;j<9;j++)
	 zone[j]=minRange;
      for(j=0;j<9;j++){
      for (i = 0+(int)(j*delta); i < (int)((j+1)*delta); i++)
      { // range are stored as an integer in current units
	r = data->getRangeMeter(i);
        if (r >= 0.020)
        { // less than 20 units is a flag value for URG scanner
          if (r<zone[j])
	     zone[j]=r;
        }
      }
      }

      // INSERT NEW CODE HERE!!!!!!
      vector<double> xLaser;
      vector<double> yLaser;

      for (int i=0; i<501; i++) {
        r = data->getRangeMeter(i);
      }

      /* SMRCL reply format */
      //snprintf(reply, MRL, "<laser l10=\"%g\" l11=\"%g\" l12=\"%g\" l13=\"%g\" l14=\"%g\" "
      //                            "l15=\"%g\" l16=\"%g\" l17=\"%g\" l18=\"%g\" />\n",
      snprintf(reply, MRL, "<laser l1=\"%g\" l1=\"%g\" l2=\"%g\" l3=\"%g\" l4=\"%g\" "
                                  "l5=\"%g\" l6=\"%g\" l7=\"%g\" l8=\"%g\" />\n",
	                  zone[0],zone[1],zone[2],zone[3],zone[4],
                           zone[5],zone[6],zone[7],zone[8]);
      // send this string as the reply to the client
      sendMsg(msg, reply);
      // save also as gloabl variable
      for(i = 0; i < 9; i++)
        var_zone->setValued(zone[i], i);
    }
    else
      sendWarning(msg, "No scandata available");
  }
  // return true if the function is handled with a positive result
  // used if scanpush or push has a count of positive results
  return true;
}

void boxDetection::createBaseVar()
{ // add also a global variable (global on laser scanner server) with latest data
  var_zone = addVarA("zone", "0 0 0 0 0 0 0 0 0", "d", "Value of each laser zone. Updated by zoneobst.");
}
