#include "ulisse_ctrl/Nurbs.h"
#include <jsoncpp/json/json.h>

Nurbs::Nurbs(int dim)
    : dim_{ dim }
    , currentParvalue_{ 0.0 }
    , isEndPath_{ false }
{
    k_ = dim + 1;
    aepsge_ = 0.001;
    aepsco_ = 0.000001;
    maxLookupParvalue_ = 0.5;
    startP_ = Eigen::VectorXd::Zero(dim_);
    endP_ = Eigen::VectorXd::Zero(dim_);
    startingD_ = Eigen::VectorXd::Zero(dim_);
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
            std::cout << order << std::endl;

            std::shared_ptr<double[]> weights(new double[obj["weigths"].size()]); //whight vector of curve.
            //Acquired the weights
            for (Json::ArrayIndex i = 0; i < obj["weigths"].size(); i++) {
                weights[count] = obj["weigths"][i].asDouble();
            }

            std::shared_ptr<double[]> coef(new double[obj["points"].size() * 4]); //Vertices of curve
            // //Acquired the vertices
            count = 0;
            double x = 0.0, y = 0.0;
            for (Json::ArrayIndex i = 0; i < obj["points"].size(); i++) {
                x = obj["points"][i][0].asDouble();
                y = obj["points"][i][1].asDouble();

                coef[count] = x * weights[i];
                coef[count + 1] = y * weights[i];
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
            SISLCurve* insert_curve = newCurve(static_cast<int>(obj["points"].size()), order + 1, knots.get(), coef.get(), kind, dim_, copy);

            if (!insert_curve) {
                std::cout << "Something Goes Wrong in NURBS Parsing" << std::endl;
                return false;
            }

            if (reverse) {
                // Turn the direction of a curve by reversing the ordering of the coefficients
                s1706(insert_curve);
            }

            nurbs_.push_back(insert_curve);
        }

        // Revert the nurbs curve
        if (reverse) {
            std::reverse(nurbs_.begin(), nurbs_.end());
        }

    } catch (Json::Exception& e) {
        // output exception information
        std::cout << "NURBS Descriptor Error: " << e.what();
        return false;
    }

    //Once acquired the nurbs vector we want to compute the starting/ending point of the path and the starting direction

    //compute the starting point and the starting direction
    Eigen::VectorXd deriveStart;
    SISLCurve* initialCurve = nurbs_[0];
    double parvalueS = 0.0; // The parameter value at which to compute position and derivatives.
    if (!ComputeDerive(initialCurve, 1, parvalueS, deriveStart))
        return false;

    //first dim components of derive are the components of the position vector, then the dim components of the tangent vector
    for (int i = 0; i < dim_; i++) {
        startP_[i] = deriveStart[i];
        startingD_[i] = deriveStart[i + 3];
    }

    //compute the starting point and the starting direction
    Eigen::VectorXd deriveEnd;
    SISLCurve* endCurve = nurbs_[nurbs_.size() - 1];
    double parvalueE = 0.0; // The parameter value at which to compute position and derivatives.
    if (!ComputeDerive(endCurve, 0, parvalueE, deriveEnd))
        return false;

    //first dim components of derive are the components of the position vector, then the dim components of the tangent vector
    for (int i = 0; i < dim_; i++) {
        endP_[i] = deriveEnd[i];
    }

    return true;
}

bool Nurbs::ComputeNextPoint(const Eigen::VectorXd& currentP, Eigen::VectorXd& nextP)
{
    //Get the current Parvalue
    if (!ComputeParameterValue(currentP))
        return false;

    if (currentParvalue_ >= nurbs_.size()) {
        isEndPath_ = true;

    } else {
        double currentParvalue = currentParvalue_;

        unsigned int indexCurrentCurve = static_cast<unsigned int>(floor(currentParvalue_));

        SISLCurve* currentCurve = nurbs_[indexCurrentCurve];

        if (currentParvalue > 1) {
            currentParvalue -= indexCurrentCurve;
        }

        int der = 0; //compute the position and the first derivative
        //    Eigen::VectorXd currentDerive;
        //    ComputeDerive(currentCurve, der, currentParvalue, currentDerive);

        // Estimate curve length
        int stat;
        double curveLenght;
        s1240(currentCurve, aepsge_, &curveLenght, &stat);

        if (stat < 0) {
            return false;
        }

        double delta_increment = (delta_ / curveLenght);

        double nextParvalue = currentParvalue + delta_increment;

        unsigned int indexNextCurve = indexCurrentCurve;
        SISLCurve* nextCurve;

        if (nextParvalue > 1) {
            if (indexCurrentCurve == nurbs_.size() - 1) {
                nextParvalue = 1;
            } else {
                nextParvalue = nextParvalue - 1;
                indexNextCurve++;
            }
        }

        nextCurve = nurbs_[indexCurrentCurve];
        ComputeDerive(nextCurve, der, nextParvalue, nextP);
    }

    return 0;
}

bool Nurbs::ComputeDerive(SISLCurve* curve, const int der, const double parvalue, Eigen::VectorXd& derive)
{
    int deriveDim = dim_ * (der + 1);
    std::cout << "DEBUG  dim " << dim_ << std::endl;
    std::cout << "DEBUG " << deriveDim << std::endl;
    derive.setZero(deriveDim);

    auto deriveTmp = std::unique_ptr<double[]>(new double[static_cast<unsigned int>(deriveDim)]);

    // S1227 is a method for computing the position and the first derivatives of the curve at  a given parameter value Evaluation from the left hand side
    int leftKnot; //Pointer to the interval in the knot vector where parvalue is located.
    int stat; /* Status messages
                > 0 : warning
                = 0 : ok
                < 0 : error*/
    std::cout << "DEBUG before" << std::endl;
    // S1227 is a method for computing the position and the first derivatives of the curve at  a given parameter value Evaluation from the left hand side
    s1227(curve, der, parvalue, &leftKnot, deriveTmp.get(), &stat);

    if (stat < 0) {
        std::cerr << "Compute inizial derive fails" << std::endl;
        return -1;
    } else {
        for (int i = 0; i < deriveDim; i++) {
            derive[i] = deriveTmp[i];
        }

        std::cout << "DEBUG after" << std::endl;
        return 0;
    }
}

bool Nurbs::ComputeParameterValue(const Eigen::VectorXd& epoint)
{
    //convert epoint in a formata readable for SISL
    auto epointTmp = std::unique_ptr<double[]>(new double[static_cast<unsigned int>(epoint.size())]);
    for (int i = 0; i < epoint.size(); i++)
        epointTmp[static_cast<unsigned long>(i)] = epoint[i];

    unsigned long numberCurves = nurbs_.size();
    double minParvalue = currentParvalue_ + maxLookupParvalue_;
    double maxParvalue = currentParvalue_ + maxLookupParvalue_;

    //to avoid in the last curve to find the parvalue outside the curve
    if (maxParvalue > numberCurves) {
        maxParvalue = numberCurves;
    }

    unsigned long indexCurrentCurve = static_cast<unsigned long>(floor(minParvalue));

    //get the curve at the current parvalue
    SISLCurve* curve = nurbs_[indexCurrentCurve];

    if (floor(maxParvalue) == floor(minParvalue) || (floor(maxParvalue) == numberCurves)) {
        double decMinParvalue, decMaxParvalue;
        std::modf(minParvalue, &decMinParvalue);
        std::modf(maxParvalue, &decMaxParvalue);

        SISLCurve* newCurve;
        int stat = 0; //Status messages
        double dist = 0.0; // The closest distance between curve and point.

        // To pick one part of a curve and make a new curve of the part
        s1713(curve, decMinParvalue, decMaxParvalue, &newCurve, &stat);
        if (stat < 0) {
            return false;
        }

        // Find the closest point between a curve and a point
        s1957(newCurve, epointTmp.get(), 3, aepsco_, aepsge_, &currentParvalue_, &dist, &stat);
        if (stat < 0) {
            return false;
        }

        currentParvalue_ = currentParvalue_ + floor(minParvalue);

    } else {
        // To select the last part of first curve, from currentParvalue to 1.
        double decMinParvalue, decMaxParvalue;
        double parvalueTmp, parvalueTmp2;

        std::modf(minParvalue, &decMinParvalue);
        decMaxParvalue = 1.0;

        SISLCurve* newCurve;
        int stat = 0; //Status messages
        double dist = 0.0; // The closest distance between curve and point.

        // To pick one part of a curve and make a new curve of the part
        s1713(curve, decMinParvalue, decMaxParvalue, &newCurve, &stat);
        if (stat < 0) {
            return false;
        }

        // Select the second curve
        SISLCurve *curve2 = nurbs_[indexCurrentCurve + 1], *newCurve2;
        decMinParvalue = 0.0;
        std::modf(maxParvalue, &decMaxParvalue);

        double dist2 = 0.0; // The closest distance between curve and point.

        // To select the first part of the second curve, from 0.0 to maxParvalue
        s1713(curve2, decMinParvalue, decMaxParvalue, &newCurve2, &stat);
        if (stat < 0) {
            return false;
        }

        // Find the closest point between the first curve and the point
        s1957(curve, epointTmp.get(), dim_, aepsco_, aepsge_, &parvalueTmp, &dist, &stat);
        if (stat < 0) {
            return false;
        }

        // Find the closest point between the second curve and the point
        s1957(curve2, epointTmp.get(), dim_, aepsco_, aepsge_, &parvalueTmp2, &dist2, &stat);
        if (stat < 0) {
            return false;
        }

        //The parameter value at of the closest point
        if (dist < dist2) {
            currentParvalue_ = parvalueTmp + floor(minParvalue);
        } else {
            currentParvalue_ = parvalueTmp2 + floor(maxParvalue);
        }
    }

    return 0;
}
