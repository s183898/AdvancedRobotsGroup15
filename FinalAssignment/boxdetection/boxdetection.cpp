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
#include "iostream"
#include "string.h"
#include "fstream"

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

bool boxDetection::setResource(UResBase * resource, bool remove) {
  bool result = true;

  if (resource->isA(UResPoseHist::getOdoPoseID())) {
    if (remove)
    poseHist = NULL;
    else if (poseHist != (UResPoseHist *) resource)
    poseHist = (UResPoseHist *) resource;
    else
    result = false;
  }

  result = UFunctionBase::setResource(resource, remove);
  return result;
}


// My variables
int vectorBreakPoint;
bool splitLine;
std::vector <double> line1_x, line1_y;
std::vector <double> line2_x, line2_y;
std::vector <double> poseL;
std::vector <double> vertex1(2), vertex2(2), vertex3(2), vertex4(2);
std::vector <double> vertices;
std::vector <double> center_L(2); // first output of this plugin

vector<double> boxDetection::transform(const vector<double>& pose_l, double &x, double &y){

  double th_lw = pose_l[2];

  double tempx = cos(th_lw)*x - sin(th_lw)*y + pose_l[0];
  double tempy = sin(th_lw)*x + cos(th_lw)*y + pose_l[1];

  vector<double> P_w;
  P_w.reserve(2);
  P_w.push_back(tempx);
  P_w.push_back(tempy);

  return P_w;
}

vector<double> boxDetection::findAVertex(vector <double>line_x, vector <double>line_y, vector<double> pose){
  double Ax, Ay;
  Ax = *line_x.begin();
  Ay = *line_y.begin();

  vector<double> vertex = transform(poseL, Ax, Ay);
  return vertex;
}

vector<double> boxDetection::findBVertex(vector <double>line_x, vector <double>line_y, vector<double> pose){
  double Bx, By;
  Bx = *line_x.rbegin();
  By = *line_y.rbegin();

  vector<double> vertex = transform(poseL, Bx, By);
  return vertex;
}


void boxDetection::linesDetection (vector <double>lines_x, vector <double>lines_y){
  vectorBreakPoint = -1;

  for (std::vector<float>::size_type i = 1; i < lines_x.size()-1; i++){

    double first_element = lines_x[i - 1];
    double second_element = lines_x[i];
    double third_element = lines_x[i + 1];

    bool bool1 = first_element > second_element;
    bool bool2 = second_element > third_element;
    bool bool3 = first_element < second_element;
    bool bool4 = second_element < third_element;

    if ((bool1 == true && bool2 == true) || (bool3 == true && bool4 == true)){
      if (i == lines_x.size() - 2){
        vectorBreakPoint = -2;
        splitLine = false;
      }
    } else {
      vectorBreakPoint = i;
      splitLine = true;
      break;
    }
  }

  if (vectorBreakPoint == -2 && splitLine == false){

    for (std::vector<float>::size_type i = 1; i < lines_y.size()-1; i++){

      double first_element = lines_y[i - 1];
      double second_element = lines_y[i];
      double third_element = lines_y[i + 1];

      bool bool1 = first_element > second_element;
      bool bool2 = second_element > third_element;
      bool bool3 = first_element < second_element;
      bool bool4 = second_element < third_element;

      if ((bool1 == true && bool2 == true) || (bool3 == true && bool4 == true)){
        if (i == lines_y.size() - 2){
          vectorBreakPoint = -3;
          splitLine = false;
        }
      } else {
        vectorBreakPoint = i;
        splitLine = true;
        break;
      }
    }
  }

  if (vectorBreakPoint > 0 && splitLine == true){
    line1_x.assign (lines_x.begin (),lines_x.begin () + vectorBreakPoint + 1);
    line1_y.assign (lines_y.begin (),lines_y.begin () + vectorBreakPoint + 1);
    line2_x.assign (lines_x.begin () + vectorBreakPoint + 1,lines_x.end ());
    line2_y.assign (lines_y.begin () + vectorBreakPoint + 1,lines_y.end ());

    // Assign the first and last point to the verteces for both the detected lines
    vertex1 = findAVertex(line1_x, line1_y, poseL);
    vertex2 = findBVertex(line1_x, line1_y, poseL);
    vertex3 = findAVertex(line2_x, line2_y, poseL);
    if (segmentSize( {vertex2[0],vertex3[0]},{vertex2[1],vertex3[1]}) < 0.10){
      double xv2 = (vertex2[0]+vertex3[0])/2;
      double yv2 = (vertex2[1]+vertex3[1])/2;
      vertex2 = {xv2,yv2};
      vertex3 = findBVertex(line2_x, line2_y, poseL);

      vertices = {vertex1[0], vertex1[1],vertex2[0], vertex2[1],vertex3[0], vertex3[1]};

    } else {
      vertex4 = findBVertex(line2_x, line2_y, poseL);
    }
  }
  if (vectorBreakPoint == -3 && splitLine == false){
    line1_x = lines_x;
    line1_y = lines_y;

    vertex1 = findAVertex(line1_x, line1_y, poseL);
    vertex2 = findBVertex(line1_x, line1_y, poseL);
    vertices = {vertex1[0], vertex1[1],vertex2[0], vertex2[1]};

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

  return size;
}


vector<double> boxDetection::lsqline(const vector<double>& x, const vector<double>& y)
{
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

  a = atan2(sin(a),cos(a));

  vector<double> line;
  line.reserve(2);
  line.push_back(a);
  line.push_back(r);

  return line;
}


vector<double> boxDetection::transformL2W(vector<double> &pose_w)
{
  vector<double> pose;
  pose.reserve(3);

  pose[0] = cos(pose_w[2]) * 0.26 + pose_w[0];
  pose[1] = sin(pose_w[2]) * 0.26 + pose_w[1];
  pose[2] = pose_w[2];


  return pose;
}

double boxDetection::findOrientation(vector<double> v, int n){
  /*
  * This function finds the orientation of the figure based on the position of vertices;
  * in both case it takes the orientation of the second longest line detected starting from the vertices;
  *
  * INPUT:   vector<douvle> v = vertices
  *
  * */
  double orientation;
  double size = 0; // size of the side of the triangle

  if (n==4){
    //Here we find the orientation of the rectangle, since it's simmetric we don't care which way the long side faces.
    for(std::vector<float>::size_type i = 0; i < v.size()-1 ; i += 2){
      for(std::vector<float>::size_type k = i+2; k < v.size()-1 ; k += 2){
        if (segmentSize( { v[i],v[k] } , { v[i+1],v[k+1] } ) > 0.25 && size == 0 ){
          double deltaX =  v[i]-v[k];
          double deltaY =  v[i+1]-v[k+1];
          orientation = atan2(deltaY, deltaX);
          size = segmentSize( { v[i],v[k] } , { v[i+1],v[k+1] });
        } else if (size > 0 && segmentSize( { v[i],v[k] } , { v[i+1],v[k+1] } ) < size && segmentSize( { v[i],v[k] } , { v[i+1],v[k+1] }) > 0.25 )  {
          double deltaX =  v[i]-v[k];
          double deltaY =  v[i+1]-v[k+1];
          orientation = atan2(deltaY, deltaX);
          size = segmentSize( { v[i],v[k] } , { v[i+1],v[k+1] });
        }
      }
    }
  } else if (n==3){
    // Here we are searching for the orientation of the longer cathetus of the triangle, starting from the common vertex with shortes cathetus.
    // In order to do this, first we find the short cathetus, which will be the short side of the triangle. Since in both cases this wont go over 15cm we pick the side that has
    // lenght less than 21cm.

    // Later we will find the second longest side (has we have done with the rectangle) and then find the common vertex so that we are sure to evaluate the correct orientation:
    // that means starting from the 90° angle
    std::vector<double> A,B;
    for(std::vector<float>::size_type i = 0; i < v.size()-1 ; i += 2){
      for(std::vector<float>::size_type k = i+2; k < v.size()-1 ; k += 2){
        if (segmentSize( { v[i],v[k] } , { v[i+1],v[k+1] } ) < 0.21 && size == 0 ){
          A.push_back(v[i]);
          A.push_back(v[i+1]);
          B.push_back(v[k]);
          B.push_back(v[k+1]);
          break;
        }
      }
    }
    for(std::vector<float>::size_type i = 0; i < v.size()-1 ; i += 2){
      for(std::vector<float>::size_type k = i+2; k < v.size()-1 ; k += 2){
        if (segmentSize( { v[i],v[k] } , { v[i+1],v[k+1] } ) > 0.25 && size == 0 ){
          if (segmentSize( { v[i],v[k] } , { v[i+1],v[k+1] } ) > 0.25 && size == 0 ){
            // if one is in common with A or B then we use this one, otherwise we continue iterating.
            if( ( v[i] == A[0] && v[i+1] == A[1] )  || ( v[i] == B[0] && v[i+1] == B[1] ) ) {
              double deltaX =  v[k]-v[i];
              double deltaY =  v[k+1]-v[i+1];
              orientation = atan2(deltaY, deltaX);
            } else if( (  v[k] == A[0] && v[k+1] == A[1] ) || ( v[k] == B[0] && v[k+1] == B[1] )) {
              double deltaX =  v[i]-v[k];
              double deltaY =  v[i+1]-v[k+1];
              orientation = atan2(deltaY, deltaX);
            }
          }
          size = segmentSize( { v[i],v[k] } , { v[i+1],v[k+1] });
        } else if (size > 0 && segmentSize( { v[i],v[k] } , { v[i+1],v[k+1] } ) < size && segmentSize( { v[i],v[k] } , { v[i+1],v[k+1] }) > 0.25 )  {
          // if one is in common with A or B then we use this one, otherwise we continue iterating.
          if( ( v[i] == A[0] && v[i+1] == A[1]) || ( v[i] == B[0] && v[i+1] == B[1]) ) {
            double deltaX =  v[k]-v[i];
            double deltaY =  v[k+1]-v[i+1];
            orientation = atan2(deltaY, deltaX);
          } else if(( v[k] == A[0] && v[k+1] == A[1]) || (v[k] == B[0] && v[k+1] == B[1])) {
            double deltaX =  v[i]-v[k];
            double deltaY =  v[i+1]-v[k+1];
            orientation = atan2(deltaY, deltaX);
          }
          size = segmentSize( { v[i],v[k] } , { v[i+1],v[k+1] });
        }
      }
    }
  }


  return orientation;
}

vector<double> boxDetection::clean (std::vector<double> vector){
  // This function eliminates the zeroes from the vector of vertices
  std::vector<double> v;

  for(std::vector<float>::size_type i = 0; i < vector.size()-1 ; i += 2){
    if(vector[i] != 0){
      v.push_back(vector[i]);
      v.push_back(vector[i+1]);
    }
  }
  return v;
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
  bool firstScan = false;
  bool secondScan = false;
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

  double zone[9];
  // check for parameters - one parameter is tested for - 'help'
  ask4help = msg->tag.getAttValue("help", value, MVL);
  firstScan = msg->tag.getAttValue("first", value, MVL);
  secondScan = msg->tag.getAttValue("second", value, MVL);

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
  else if (firstScan) //------------------------------------------ MY CODE HERE ------------------------
  {
    // Get laser data
    data = getScan(msg, (ULaserData *)extra);

    // check if data is valid
    if (data->isValid())
    {
      // create vector with robot pose (x,y,th) in world given by the caller
      vector<double> poseW;
      poseW.reserve(3);
      poseW.push_back(xr);
      poseW.push_back(yr);
      poseW.push_back(thr);
      // find the pose of the laser scanner in world
      poseL = transformL2W(poseW);

      // vectors to store x,y data (point in the laser frame)
      vector<double> x;
      vector<double> y;

      for (int i = 0; i < data->getRangeCnt(); i++)
      {
        double range = data->getRangeMeter(i);
        double angle = data->getAngleRad(i);

        // x,y data in the laser frame
        double xx = cos(angle) * range;
        double yy = sin(angle) * range;

        // transformation to world frame
        vector<double> P_w = transform(poseL, xx, yy);
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
        linesDetection(x,y);

        // saving the vertices to a file ----------------------------------------
        std::ofstream firstScan("FirstScan.txt");

        for(std::vector<float>::size_type i = 0; i < vertices.size() ; i++) {
          std::string vertix = to_string(vertices[i]);
          firstScan << vertix << endl; // I also tried replacing endl with a "\n!
        }

        // Close the file
        firstScan.close();

      }
    }
  }
  else if (secondScan) //----------------------------------------------------------SECOND SCAN-------------------------------
  {
    // Get laser data
    data = getScan(msg, (ULaserData *)extra);

    // check if data is valid
    if (data->isValid())
    {
      // create vector with robot pose (x,y,th) in world given by the caller
      vector<double> poseW;
      poseW.reserve(3);
      poseW.push_back(xr);
      poseW.push_back(yr);
      poseW.push_back(thr);
      // find the pose of the laser scanner in world
      poseL = transformL2W(poseW);

      // vectors to store x,y data (point in the laser frame)
      vector<double> x;
      vector<double> y;

      for (int i = 0; i < data->getRangeCnt(); i++)
      {
        double range = data->getRangeMeter(i);
        double angle = data->getAngleRad(i);

        // x,y data in the laser frame
        double xx = cos(angle) * range;
        double yy = sin(angle) * range;

        // transformation to world frame
        vector<double> P_w = transform(poseL, xx, yy);
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
        linesDetection(x,y);

        // Opening the file with the vertices-------------
        std::ifstream pastData("FirstScan.txt");
        std::vector <double>  FirstScanVector;

        double raw;

        if (! pastData.is_open()){
          cout << "Error opening file";
        } else {
          while (pastData >> raw){
            FirstScanVector.push_back(raw);
          }
        }
        pastData.close();

        std::vector<double> finalVect, finalV;
        finalVect.reserve( FirstScanVector.size() + vertices.size());
        finalVect.insert(finalVect.end(), vertices.begin(), vertices.end());
        finalVect.insert(finalVect.end(), FirstScanVector.begin(), FirstScanVector.end());

        // Removing the duplicates from the vector so that we have only a vector with every vertex repeated only once!
        for(std::vector<float>::size_type i = 0; i < finalVect.size()-1 ; i += 2){
          for(std::vector<float>::size_type k = i+2; k < finalVect.size()-1 ; k += 2){
            if(segmentSize( { finalVect[i],finalVect[k] } , { finalVect[i+1],finalVect[k+1] })< 0.1){
              if(finalVect[i] == 0 and finalVect[i+1]==0){
                break;
              }
              finalVect[k] = 0;
              finalVect[k+1] = 0;
              break;
            }
          }
        }

        vertices.clear();

        vertices = clean(finalVect);

        int num = vertices.size()/2;

        if(num == 3){
          cout << "\nI found a triangle\n";
          double xs, ys;
          for(std::vector<float>::size_type i = 0; i < vertices.size()-1 ; i += 2){
            xs += vertices[i];
            ys += vertices[i+1];
          }
          xs = xs/3;
          ys = ys/3;
          cout << "\nThe center of the triangle has coordintes:\nx= " << xs << "\ny= " << ys;
          double orientation = findOrientation(vertices, num);

          cout << "\nOrientation of the triangle is: " << orientation;

        } else if (num == 4){
          cout << "\n I found a rectangle\n";
          double xs, ys;
          for(std::vector<float>::size_type i = 0; i < vertices.size()-1 ; i += 2){
            xs += vertices[i];
            ys += vertices[i+1];
          }
          xs = xs/4;
          ys = ys/4;
          cout << "\nThe center of the rectangle has coordintes:\nx= " << xs << "\ny= " << ys;
          double orientation = findOrientation(vertices, num);

          cout << "\nOrientation of the rectangle is: " << orientation;
        } else {
          cout << "\n Impossible to determine the shape of the figure\n";
        }


        for(std::vector<float>::size_type i = 0; i < vertices.size()-1 ; i += 2){
          for(std::vector<float>::size_type k = 0; k < FirstScanVector.size()-1 ; k += 2){
            if (k ==FirstScanVector.size()-2 && (segmentSize( {vertices[i],FirstScanVector[k]},{vertices[i+1],FirstScanVector[k+1]}) >= 0.10)){
              finalV.push_back(vertices[i]);
              finalV.push_back(vertices[i+1]);
            }
          }
        }

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
