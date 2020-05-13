#include "ulisse_ctrl/Nurbs.h"
#include <jsoncpp/json/json.h>

Nurbs::Nurbs(int dim)
    : dim_{ dim }
    , Parvalue_{ 0.0 }
    , currentParvalue_{ 0.0 }
    , nextParvalue_{ 0.0 }
    , isEndPath_{ false }
{
    k_ = dim + 1;
    aepsge_ = 0.001;
    aepsco_ = 0.000001;
    maxLookupParvalue_ = 0.5;
    startingD_ = Eigen::VectorXd::Zero(dim_);
    delta_ = 2.0;
    maxLookupParvalue_ = 0.5;
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

    std::cout << "currentParvalue: " << Parvalue_ << std::endl;

    unsigned int indexCurrentCurve = static_cast<unsigned int>(floor(Parvalue_));

    SISLCurve* currentCurve = nurbs_[indexCurrentCurve];

    currentParvalue_ = Parvalue_;

    if (currentParvalue_ > 1) {
        currentParvalue_ -= indexCurrentCurve;
    }

    int der = 1; //compute the position and the first derivative
    //    Eigen::VectorXd currentDerive;
    //    ComputeDerive(currentCurve, der, currentParvalue, currentDerive);

    // Estimate curve length
    int stat;
    double curveLenght;
    s1240(currentCurve, aepsge_, &curveLenght, &stat);

    std::cout << "Current curve lunght " << curveLenght << std::endl;
    std::cout << "delta " << delta_ << std::endl;

    if (stat < 0) {
        std::cerr << "ComputeNextPoint: s1240 fails" << std::endl;
        return false;
    }

    double delta = delta_ / curveLenght;

    nextParvalue_ = currentParvalue_ + delta;

    std::cout << "nextParvalue: " << nextParvalue_ << std::endl;

    unsigned int indexNextCurve = indexCurrentCurve;
    SISLCurve* nextCurve = nullptr;

    if (nextParvalue_ > 1) {
        if (indexCurrentCurve == nurbs_.size() - 1) {
            nextParvalue_ = 1;
        } else {
            nextParvalue_ += -1;
            indexNextCurve++;
        }
    }

    nextCurve = nurbs_[indexNextCurve];
    Eigen::VectorXd derive;
    if (!ComputeDerive(nextCurve, der, nextParvalue_, derive)) {
        std::cerr << "ComputeNextPoint: ComputeDerive fails" << std::endl;
        return false;
    }

    ctb::Cartesian2MapPoint(derive, centroid_, nextP);

    std::cout << "next point on curve: " << nextP.latitude << nextP.longitude << std::endl;

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
    double minParvalue = Parvalue_;
    double maxParvalue = Parvalue_ + maxLookupParvalue_;

    //to avoid in the last curve to find the parvalue outside the curve
    if (maxParvalue > numberCurves) {
        maxParvalue = numberCurves;
    }

    unsigned long indexCurrentCurve = static_cast<unsigned long>(floor(Parvalue_));

    //get the curve at the current parvalue
    SISLCurve* curve = nurbs_[indexCurrentCurve];

    if (floor(maxParvalue) == floor(minParvalue) || (floor(maxParvalue) == numberCurves)) {
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
        s1957(newCurve, epointTmp.get(), 3, aepsco_, aepsge_, &Parvalue_, &dist, &stat);
        if (stat < 0) {
            std::cerr << "ComputesParameterValue: s1957 fails" << std::endl;
            return false;
        }

        Parvalue_ = Parvalue_ + floor(minParvalue);

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
        s1957(curve, epointTmp.get(), dim_, aepsco_, aepsge_, &parvalueTmp, &dist, &stat);
        if (stat < 0) {
            std::cerr << "ComputesParameterValue: s1957 fails" << std::endl;
            return false;
        }

        // Find the closest point between the second curve and the point
        s1957(curve2, epointTmp.get(), dim_, aepsco_, aepsge_, &parvalueTmp2, &dist2, &stat);
        if (stat < 0) {
            std::cerr << "ComputesParameterValue: s1957 fails" << std::endl;
            return false;
        }

        //The parameter value at of the closest point
        if (dist < dist2) {
            Parvalue_ = parvalueTmp + floor(minParvalue);
        } else {
            Parvalue_ = parvalueTmp2 + floor(maxParvalue);
        }
    }

    return true;
}
