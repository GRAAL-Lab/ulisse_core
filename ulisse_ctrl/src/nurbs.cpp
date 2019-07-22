#include <iomanip>

#include "rclcpp/rclcpp.hpp"

#include "ulisse_ctrl/ctrl_data_structs.hpp"
#include "ulisse_ctrl/fsm_defines.hpp"

#include "rml/RML.h"
#include <cmath>

#include "sisl.h"
#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept>

using namespace ulisse;


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("nurbs");

    int rate = 10;
    rclcpp::WallRate loop_rate(rate);

    const int order = 4;
    int dimension = 3;
    int degree = 3;
    int cv_count = 4;
    int knot_count = 6;


    float w =1.0/3;

    double coef[] = {1,   0, 0,  1,
                     1*w,   2*w, 0, w,
                     -1*w,  2*w, 0, w,
                     -1,  0, 0, 1};

    double knots[] = {0, 0, 0, 0, 1, 1, 1, 1};

    try {

        SISLCurve* curve = newCurve(cv_count,       // number of control points
                                    order,          // order of spline curve (degree + 1)
                                    knots,          // pointer to knot vector (parametrization)
                                    coef,           // pointer to coefficient vector (control points)
                                    2,              // kind => 2 : NURBS curve
                                    3,              // dimension
                                    2);             // no copying of information, 'borrow' arrays
        if (!curve) {
            std::cout << "ERRORE" << std::endl;
        }
        else{
            std::cout << "YEEEEEEEEEE" << std::endl;
        }

        int leftknot, stat;
        double derive[4];
        // Compute the position and the left-hand derivatives of a curve at a given parameter value.
        s1227(curve, 0, 0.0, &leftknot, derive, &stat);
        std::cout << "POINT AT 0.0 : ( " << derive[0] << " , " << derive[1] << " , " << derive[2] << " ) " << std::endl;

        s1227(curve, 0, 1.0, &leftknot, derive, &stat);
        std::cout << "POINT AT 1.0 : ( " << derive[0] << " , " << derive[1] << " , " << derive[2] << " ) " << std::endl;

        s1227(curve, 0, 0.5, &leftknot, derive, &stat);
        std::cout << "POINT AT 0.5 : ( " << derive[0] << " , " << derive[1] << " , " << derive[2] << " ) " << std::endl;

        SISLCurve* newcurve;
        SISLCurve* newcurve2;
        // To pick one part of a closed curve and make a new curve of that
        // part. If the routine is used on an open curve and endpar ≤ begpar,
        // the last part of the curve is translated so that the end of the curve
        // joins the start.
        s1713(curve, 0.5, 0.8, &newcurve, &stat);
        s1713(curve, 0.8, 0.9, &newcurve2, &stat);

        SISLCurve* result_curve;
        // To join two curves at the ends that lie closest to each other, if
        // the distance between the ends is less than the tolerance epsge. If
        // curve1 is to be joined at the start, the direction of the curve is
        // turned. If curve2 is to be joined at the end, the direction of this
        // curve is turned. This means that curve1 always makes up the first
        // part of the new curve. If epsge is positive, but smaller than the
        // smallest distance between the ends of the two curves, a NULL
        // pointer is returned.
        s1716(newcurve, newcurve2, -1, &result_curve, &stat);

        s1221(result_curve, 0, 0.0, &leftknot, derive, &stat);
        std::cout << "RESULT CURVE => POINT AT 0.0 : ( " << derive[0] << " , " << derive[1] << " ) " << std::endl;

        // Turn the direction of a curve by reversing the ordering of the
        // coefficients. The start parameter value of the new curve is the
        // same as the start parameter value of the old curve. This routine
        // turns the direction of the orginal curve. If you want a copy with
        // a turned direction, just make a copy and turn the direction of the
        // copy.
        s1706(curve);

        // Compute the position and the right-hand derivatives of a curve at a given parameter value
        s1221(curve, 0, 0.0, &leftknot, derive, &stat);
        std::cout << "REVERSING POINT AT 0.0 : ( " << derive[0] << " , " << derive[1] << " ) " << std::endl;

        // Find the closest point between a curve and a point. The method is
        // fast and should work well in clear cut cases but does not guarantee
        // finding the right solution. As long as it doesn’t fail, it will find
        // exactly one point. In other cases, use s1953().
        double epoint[3];
        epoint[0] = 0;
        epoint[1] = 2;
        epoint[2] = 0;
        double aepsco = 0.01;
        double aepsge = 0.01;

        double gpar = 0;
        double dist = 0;
        int jstat = 0;
        s1957(curve, epoint, 3, aepsco, aepsge, &gpar, &dist, &jstat);
        std::cout << "POINT (0, 2) CLOSEST: PARAM: " << gpar << " , DISTANCE: " << dist << std::endl;

    } catch (std::exception& e) {
        std::cerr << "Exception thrown: " << e.what() << std::endl;
    }

    while (rclcpp::ok()) {

    }

    return 0;
}

