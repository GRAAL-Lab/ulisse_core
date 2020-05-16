#include "ulisse_ctrl/nurbs.h"
#include <jsoncpp/json/json.h>

Nurbs::Nurbs(int dim)
    : dim_{ dim }
    , startP_{ 0.0, 0.0 }
    , endP_{ 0.0, 0.0 }
    , parvalue_{ 0.0 }
    , centroid_{ 0.0, 0.0 }
{
    //default initialization of Nurbs.cpp param
    nurbsParam.aepsge = 0.001;
    nurbsParam.aepsco = 0.000001;
    nurbsParam.maxLookupParvalue = 0.5;
    nurbsParam.deltaMin = 2.0;
    nurbsParam.deltaMax = 5.0;
    nurbsParam.directionError = 0.436; //25 gradi

    //Default initialization of the currentDelta
    currentDelta_ = nurbsParam.deltaMax;
}

Nurbs::~Nurbs() { nurbs_.clear(); }

bool Nurbs::Initialization(const std::string& jasonNurbs)
{
    Json::Reader reader;
    Json::Value obj, objMaster;
    bool reverse = false;
    int count = 0;

    //parse the jason
    reader.parse(jasonNurbs, objMaster);

    // check whatever the path has beeen reverse
    reverse = objMaster["direction"].asInt() ? true : false;

    centroid_.latitude = /*44.414165*/ objMaster["centroid"][0].asDouble();
    centroid_.longitude = /* 8.942184*/ objMaster["centroid"][1].asDouble();

    //some param needs to create a new curve
    int kind = 2; /* Type of curve.
                    = 1 : Polynomial B-spline curve.
                    = 2 : Rational B-spline (nurbs) curve.
                    = 3 : Polynomial Bezier curve.
                    = 4 : Rational Bezier curve*/

    int copy = 1; /* Flag
                     = 0 : Set pointer to input arrays.
                     = 1 : Copy input arrays.
                     = 2 : Set pointer and remember to free arrays. */

    try {
        for (Json::Value c : objMaster["curves"]) {

            reader.parse(c.toStyledString(), obj);

            int order; //Order of curve.
            order = obj["degree"].asInt();

            std::shared_ptr<double[]> weights(new double[obj["weigths"].size()]); //whight vector of curve.
            //Acquired the weights
            for (Json::ArrayIndex i = 0; i < obj["weigths"].size(); i++) {
                weights[i] = obj["weigths"][i].asDouble();
            }

            std::shared_ptr<double[]> coef(new double[obj["points"].size() * 4]); //Vertices of curve
            // //Acquired the vertices
            count = 0;
            ctb::LatLong point;
            Eigen::Vector3d pointC;
            for (Json::ArrayIndex i = 0; i < obj["points"].size(); i++) {
                point.latitude = obj["points"][i][0].asDouble();
                point.longitude = obj["points"][i][1].asDouble();

                ctb::Map2CartesianPoint(point, centroid_, pointC);

                coef[count] = pointC[0] * weights[i];
                coef[count + 1] = pointC[1] * weights[i];
                coef[count + 2] = 0;
                coef[count + 3] = weights[i];

                count += 4;
            }

            std::shared_ptr<double[]> knots(new double[obj["knots"].size()]); //Knot vector of curve
            //Acquired the knots
            for (Json::ArrayIndex i = 0; i < obj["knots"].size(); i++) {
                knots[i] = obj["knots"][i].asDouble();
            }

            //create the curve
            SISLCurve* curve = newCurve(static_cast<int>(obj["points"].size()), order + 1, knots.get(), coef.get(), kind, dim_, copy);

            if (curve == nullptr) {
                std::cout << "Something Goes Wrong in NURBS Parsing" << std::endl;
                return false;
            }

            if (reverse) {
                // Turn the direction of a curve by reversing the ordering of the coefficients
                s1706(curve);
            }

            nurbs_.push_back(curve);
        }

    } catch (Json::Exception& e) {
        // output exception information
        std::cout << "NURBS Descriptor Error: " << e.what();
        return false;
    }

    // Revert the nurbs curve
    if (reverse) {
        std::reverse(nurbs_.begin(), nurbs_.end());
    }

    //Once acquired the nurbs vector we want to compute the starting/ending point of the path and the starting direction

    //compute the starting point and the starting direction
    Eigen::VectorXd deriveStart;
    SISLCurve* initialCurve = nurbs_[0];
    double parvalueS = 0.0; // The parameter value at which to compute position and derivatives.
    if (!ComputeDerive(initialCurve, 0, parvalueS, deriveStart)) {
        std::cerr << "Initialization: ComputeDerive start point fails" << std::endl;
        return false;
    }

    Eigen::VectorXd startP = Eigen::VectorXd::Zero(dim_);
    //first dim components of derive are the components of the position vector, then the dim components of the tangent vector
    for (int i = 0; i < dim_; i++) {
        startP[i] = deriveStart[i];
        //        startingD_[i] = deriveStart[i + 3];
    }
    ctb::Cartesian2MapPoint(startP, centroid_, startP_);

    Eigen::VectorXd endP = Eigen::VectorXd::Zero(dim_);
    //compute the starting point and the starting direction
    Eigen::VectorXd deriveEnd;
    SISLCurve* endCurve = nurbs_[nurbs_.size() - 1];
    double parvalueE = 1.0; // The parameter value at which to compute position and derivatives.
    if (!ComputeDerive(endCurve, 0, parvalueE, deriveEnd)) {
        std::cerr << "Initialization: ComputeDerive endpoint fails" << std::endl;
        return false;
    }

    //first dim components of derive are the components of the position vector, then the dim components of the tangent vector
    for (int i = 0; i < dim_; i++) {
        endP[i] = deriveEnd[i];
    }
    ctb::Cartesian2MapPoint(endP, centroid_, endP_);

    return true;
}

bool Nurbs::ComputeNextPoint(const ctb::LatLong& currentP, ctb::LatLong& nextP)
{
    Eigen::VectorXd currentPCartesian = Eigen::VectorXd::Zero(dim_);
    ctb::Map2CartesianPoint(currentP, centroid_, currentPCartesian);

    //Get the current Parvalue
    if (!ComputeParameterValue(currentPCartesian)) {
        std::cerr << "ComputeNextPoint: ComputeParameterValue fails" << std::endl;
        return false;
    }

    //get the index of the current curve as the floor of the current parvalue
    unsigned int indexCurrentCurve = static_cast<unsigned int>(floor(parvalue_));

    // get the current curve
    SISLCurve* currentCurve = nurbs_[indexCurrentCurve];

    //Compute the position and the first derivative of the current paramenter value
    Eigen::VectorXd derive, currenDirection = Eigen::VectorXd::Zero(dim_);
    int der = 1; //compute the position and the first derivative
    if (!ComputeDerive(currentCurve, der, parvalue_, derive)) {
        std::cerr << "ComputeNextPoint: ComputeDerive fails" << std::endl;
        return false;
    }

    //the first derivative of the current paramenter value is the direction of the curve
    currenDirection = derive.bottomRows(dim_) / derive.bottomRows(dim_).norm();
    std::cout << "tangent of the current parvalue: " << currenDirection.transpose() << std::endl;

    Eigen::VectorXd nextDir;
    Eigen::VectorXd possibleNextP;

    //To compute the next point on the curve, by addind a delta increment on the current parvalue
    //To avoiding reaching a point with a direction too different from the actual one, it has been defined
    //two threshold deltaMin and delta max. The current delta increment is
    //inizialize with deltaMax and then compute the next point and the next direction. Then,
    //the difference between the current and the next direction is evalueted. In this case, the delta is decremented
    //and a new point is compute with the new delta. This process is repeated until either a point with
    //a direction not to different form the cerrent one is found or the current delta reaches the deltaMin threshold

    //Try to compute the next point on the curve and the next direction
    if (!ComputePossibleNextPoint(nextDir, possibleNextP)) {
        std::cerr << "ComputeNextPossiblePoint fails" << std::endl;
        return false;
    }

    //Evaluete the diference between the current direction and the next direction
    Eigen::VectorXd diff = rml::ReducedVersorLemma(currenDirection, nextDir);

    //While the differece is over the threshold compute a new point with different delta
    while (diff.norm() > nurbsParam.directionError) {
        if (currentDelta_ <= nurbsParam.deltaMin) {
            //if the delta reaches the min value, set the current delta to the min value
            currentDelta_ = nurbsParam.deltaMin;
            // and compute the parvalue at this delta increment
            if (!ComputePossibleNextPoint(nextDir, possibleNextP)) {
                std::cerr << "ComputeNextPossiblePoint fails" << std::endl;
                return false;
            }
            break;
        } else {
            //otherwise decrease the delta by a 0.5 meter factor
            currentDelta_ -= 0.5;
            //compute the next parvalue with this delta increment
            if (!ComputePossibleNextPoint(nextDir, possibleNextP)) {
                std::cerr << "ComputeNextPossiblePoint fails" << std::endl;
                return false;
            }
        }
    }

    //Detect if the difference is under threshold and restore the max delta
    if (diff.norm() < nurbsParam.directionError) {
        currentDelta_ = nurbsParam.deltaMax;
    }

    //Convert the next point in latlong coordinates
    ctb::Cartesian2MapPoint(possibleNextP, centroid_, nextP);
    std::cout << "tangent of the current parvalue: " << currenDirection.transpose() << std::endl;
    std::cout << "current delta: " << currentDelta_ << std::endl;
    return true;
}

bool Nurbs::ComputePossibleNextPoint(Eigen::VectorXd& nextDirection, Eigen::VectorXd& nextP)
{
    nextP = Eigen::VectorXd::Zero(dim_);

    //get the index of the current curve as the floor of the current parvalue
    unsigned int indexCurrentCurve = static_cast<unsigned int>(floor(parvalue_));
    // get the current curve
    SISLCurve* currentCurve = nurbs_[indexCurrentCurve];

    //take the decimal part
    double intPart;
    double currentParvalue = std::modf(parvalue_, &intPart);

    // Estimate curve length
    double curveLenght;
    if (!ComputeCurveLength(currentCurve, curveLenght)) {
        std::cerr << "ComputeNextPossiblePoint: ComputeCurveLength fails" << std::endl;
        return false;
    }

    double delta = currentDelta_ / curveLenght;

    double nextParvalue = currentParvalue + delta;

    unsigned int indexNextCurve = indexCurrentCurve;
    SISLCurve* nextCurve = nullptr;

    //Check if adding a delta increment the next parvalue is on the next curve
    if (nextParvalue > 1) {
        if (indexCurrentCurve == nurbs_.size() - 1) {
            nextParvalue = 1.0;
        } else {
            //compute the delta remaining of the last curve
            double deltaRemaining = (1 - currentParvalue) * curveLenght;

            indexNextCurve++;
            SISLCurve* nextCurve = nurbs_[indexNextCurve];

            // Estimate the new curve length
            double curveLenght;
            if (ComputeCurveLength(nextCurve, curveLenght)) {
                std::cerr << "ComputeNextPossiblePoint: ComputeCurveLength fails" << std::endl;
                return false;
            }

            //compute the new delta
            delta = (currentDelta_ - deltaRemaining) / curveLenght;

            nextParvalue = delta;
        }
    }

    nextCurve = nurbs_[indexNextCurve];

    int der = 1;
    Eigen::VectorXd derive;
    if (!ComputeDerive(nextCurve, der, nextParvalue, derive)) {
        std::cerr << "ComputeNextPossiblePoint: ComputeDerive fails" << std::endl;
        return false;
    }

    nextDirection = Eigen::VectorXd::Zero(dim_);
    nextDirection = derive.bottomRows(dim_) / derive.bottomRows(dim_).norm();
    std::cout << "tangent of the next parvalue: " << nextDirection.transpose() << std::endl;

    nextP = derive.topRows(dim_);
    return true;
}

bool Nurbs::ComputeCurveLength(SISLCurve* curve, double& length)
{
    //SISL call to campute the curve length
    int stat = 0;
    s1240(curve, nurbsParam.aepsge, &length, &stat);
    if (stat < 0) {
        std::cerr << "ComputeCurveLength fails" << std::endl;
        return false;
    }
    return true;
}

bool Nurbs::ComputeDerive(SISLCurve* curve, const int der, const double parvalue, Eigen::VectorXd& derive)
{
    int deriveDim = dim_ * (der + 1);
    derive.setZero(deriveDim);

    auto deriveTmp = std::unique_ptr<double[]>(new double[static_cast<unsigned int>(deriveDim)]);

    // S1227 is a method for computing the position and the first derivatives of the curve at  a given parameter value Evaluation from the left hand side
    int leftKnot; //Pointer to the interval in the knot vector where parvalue is located.
    int stat; /* Status messages
                > 0 : warning
                = 0 : ok
                < 0 : error*/
    // S1227 is a method for computing the position and the first derivatives of the curve at  a given parameter value Evaluation from the left hand side
    s1227(curve, der, parvalue, &leftKnot, deriveTmp.get(), &stat);

    if (stat < 0) {
        std::cerr << "Compute derive fails" << std::endl;
        return -1;
    } else {
        for (int i = 0; i < deriveDim; i++) {
            derive[i] = deriveTmp[static_cast<unsigned int>(i)];
        }
    }
    return true;
}

bool Nurbs::ComputeParameterValue(const Eigen::VectorXd& epoint)
{
    std::cout << "epoint: " << epoint[0] << ", " << epoint[1] << std::endl;
    //convert epoint in a formata readable for SISL
    auto epointTmp = std::unique_ptr<double[]>(new double[static_cast<unsigned int>(epoint.size())]);
    for (int i = 0; i < epoint.size(); i++)
        epointTmp[static_cast<unsigned long>(i)] = epoint[i];

    unsigned long numberCurves = nurbs_.size();
    double minParvalue = parvalue_;
    double maxParvalue = parvalue_ + nurbsParam.maxLookupParvalue;

    //to avoid in the last curve to find the parvalue outside the curve
    if (maxParvalue > numberCurves) {
        maxParvalue = numberCurves;
    }

    unsigned long indexCurrentCurve = static_cast<unsigned long>(floor(parvalue_));

    //get the curve at the current parvalue
    SISLCurve* curve = nurbs_[indexCurrentCurve];

    if (floor(maxParvalue) == floor(minParvalue) || maxParvalue >= numberCurves) {
        double decMinParvalue, decMaxParvalue;
        double intPart;
        decMinParvalue = std::modf(minParvalue, &intPart);
        decMaxParvalue = std::modf(maxParvalue, &intPart);

        SISLCurve* newCurve = nullptr;
        int stat = 0; //Status messages
        double dist = 0.0; // The closest distance between curve and point.

        // To pick one part of a curve and make a new curve of the part
        s1713(curve, decMinParvalue, decMaxParvalue, &newCurve, &stat);
        if (stat < 0) {
            std::cerr << "ComputesParameterValue: 1713 fails" << std::endl;
            return false;
        }

        // Find the closest point between a curve and a point
        s1957(newCurve, epointTmp.get(), 3, nurbsParam.aepsco, nurbsParam.aepsge, &parvalue_, &dist, &stat);
        if (stat < 0) {
            std::cerr << "ComputesParameterValue: s1957 fails" << std::endl;
            return false;
        }

        //The parameter value of the closest point in the parameter interval of the curve
        parvalue_ = parvalue_ + floor(minParvalue);

    } else {
        // To select the last part of first curve, from currentParvalue to 1.
        double decMinParvalue, decMaxParvalue;
        double parvalueTmp, parvalueTmp2;
        double intPart;

        decMinParvalue = std::modf(minParvalue, &intPart);

        SISLCurve* newCurve = nullptr;
        int stat = 0; //Status messages
        double dist = 0.0; // The closest distance between curve and point.

        // To pick one part of a curve and make a new curve of the part
        s1713(curve, decMinParvalue, 1.0, &newCurve, &stat);
        if (stat < 0) {
            std::cerr << "ComputesParameterValue: s1713 fails" << std::endl;
            return false;
        }

        // Select the second curve
        SISLCurve *curve2 = nurbs_[indexCurrentCurve + 1], *newCurve2 = nullptr;

        decMaxParvalue = std::modf(maxParvalue, &intPart);

        double dist2 = 0.0; // The closest distance between curve and point.

        // To select the first part of the second curve, from 0.0 to maxParvalue
        s1713(curve2, 0.0, decMaxParvalue, &newCurve2, &stat);
        if (stat < 0) {
            std::cerr << "ComputesParameterValue: s1713 fails" << std::endl;
            return false;
        }

        // Find the closest point between the first curve and the point
        s1957(curve, epointTmp.get(), dim_, nurbsParam.aepsco, nurbsParam.aepsge, &parvalueTmp, &dist, &stat);
        if (stat < 0) {
            std::cerr << "ComputesParameterValue: s1957 fails" << std::endl;
            return false;
        }

        // Find the closest point between the second curve and the point
        s1957(curve2, epointTmp.get(), dim_, nurbsParam.aepsco, nurbsParam.aepsge, &parvalueTmp2, &dist2, &stat);
        if (stat < 0) {
            std::cerr << "ComputesParameterValue: s1957 fails" << std::endl;
            return false;
        }

        //The parameter value at of the closest point
        if (dist < dist2) {
            parvalue_ = parvalueTmp + floor(minParvalue);
        } else {
            parvalue_ = parvalueTmp2 + floor(maxParvalue);
        }
    }

    return true;
}
