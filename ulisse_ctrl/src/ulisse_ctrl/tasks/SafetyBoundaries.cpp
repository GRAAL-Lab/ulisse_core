#include "ulisse_ctrl/tasks/SafetyBoundaries.h"

using namespace ctb;
namespace ikcl {

SafetyBoundaries::SafetyBoundaries(std::string taskID, std::shared_ptr<rml::RobotModel> robotModel, std::string frameID)
    : tpik::InequalityTask(taskID, 3, robotModel->GetTotalDOFs(), tpik::InequalityTaskType::Decreasing)
    , robotModel_(robotModel)
    , frameID_(frameID)

{
    desiredVelocity_(0) = 1.0;
    Ai_.setZero(taskSpace_, taskSpace_);
    x_dot_.setZero(taskSpace_);
    J_.setZero(taskSpace_, DoF_);
    isBoundariesInitialized_ = false;
    alignVector_.setZero();
}

void SafetyBoundaries::SetDesiredVelocity(Eigen::Vector3d desiredVelocity)
{
    desiredVelocity_ = desiredVelocity;
}

void SafetyBoundaries::SetPose(std::shared_ptr<Eigen::Vector6d> pose)
{
    pose_ = pose;
}

const Eigen::Vector3d SafetyBoundaries::GetAlignVector()
{
    return alignVector_;
}
void SafetyBoundaries::SetControlVariable(Eigen::Vector3d x)
{
    x_ = x;
}
void SafetyBoundaries::Update() throw(tpik::ExceptionWithHow)
{
    CheckInitialization();
    if (!isBoundariesInitialized_) {
        tpik::NotInitialziedTaskParameterException jointsLimitException;
        std::string how = "[SafetyBoundaries] The boundaries have not been set, "
                          "use SetBoundaries() for task "
            + ID_;
        jointsLimitException.SetHow(how);
        throw(jointsLimitException);
    }

    Eigen::Vector3d alignVector;

    std::shared_ptr<double[]> poseEuclidian(new double[3]);
    LatLong pose;
    pose.latitude = (*pose_)(0);
    pose.longitude = (*pose_)(1);

    ctb::Map2EuclidianPoint(pose, centroid_, poseEuclidian);

    DistanceCheck(point_type(poseEuclidian[0], poseEuclidian[1]), alignVector_);

    UpdateInternalActivationFunction();
    UpdateJacobian();
    UpdateReference();
    SaturateReference();
    SaturateReferenceComponentWise();
}

void SafetyBoundaries::UpdateJacobian()
{
    J_ = robotModel_->GetCartesianJacobian(frameID_).block(3, 0, 3, DoF_);
}

void SafetyBoundaries::UpdateReference()
{
    x_dot_(0) = taskParameter_.gain * (decreasingBellShape_.xmax(0) - x_(0));
}

void SafetyBoundaries::UpdateInternalActivationFunction()
{
    Ai_(0, 0) = rml::DecreasingBellShapedFunction(decreasingBellShape_.xmin(0), decreasingBellShape_.xmax(0), 0.0, 1.0, x_(0));
    std::cout << "Debug Ai:" << std::endl;
    std::cout << Ai_ << std::endl;
}

bool SafetyBoundaries::InitializePolygon(std::string polygonString, LatLong inizialPosition)
{
    segments_.clear();
    segment_type seg;

    centroid_ = inizialPosition;
    boost::geometry::read_wkt(polygonString, poly_);

    // Generate a list with all the segments
    double x, y;

    for (auto it = boost::begin(boost::geometry::exterior_ring(poly_));
         (it + 1) != boost::end(boost::geometry::exterior_ring(poly_)); ++it) {

        x = (*it).x();

        y = (*it).y();

        point_type p(x, y);

        x = (*(it + 1)).x();
        y = (*(it + 1)).y();
        point_type next(x, y);

        MakeSegments(p, next, seg);

        segments_.push_front(seg);
    }

    isBoundariesInitialized_ = true;

    return true;
}

void SafetyBoundaries::DistanceCheck(point_type const& currentPosition, Eigen::Vector3d& alignVector)
{

    Eigen::VectorXd Ai;
    std::list<segment_type> segments, minDistsegments;
    bool isConvex = false;

    Ai.setZero(static_cast<long int>(segments_.size()));

    alignVector.setZero();

    // copy the list of all the segments of the polygon
    segments = segments_;

    // take the two nearest segments
    ExtractMinDistanceSegments(segments, currentPosition, minDistsegments);

    // check if the two nearest segments are a convex side of the polygon or not
    isConvex = IsConvex(minDistsegments);

    std::cout << "DEBUG:: Is Convex:" << isConvex << std::endl;

    // Depending on the type of the two nearest segments, the strategy to compute
    // the aligment vector to cameback to safty position is different.

    // check if the robot is near the two segment convex
    if (isConvex) {
        // compute the direct of alignent to escape from the border in case of
        // convex side of the polygon
        ComputeAlignVectorConvex(minDistsegments, currentPosition, alignVector);
    }
    // check if the robot is near the two segment concav
    else {
        // compute the direct of alignent to escape from the border in case of
        // concave side of the polygon
        ComputeAlignVectorConcave(minDistsegments.front(), currentPosition, alignVector);
    }

    std::cout << "DEbug alignvector:" << alignVector << std::endl;
}

void SafetyBoundaries::ComputeAlignVectorConcave(segment_type segment, point_type currentPosition, Eigen::Vector3d& alignVector)
{
    // there are three situation in which the robot can be:

    // starting point of the first segment the one at min dist
    point_type p1{ boost::geometry::get<0, 0>(segment), boost::geometry::get<0, 1>(segment) };

    // ending point of the first segment
    point_type p2{ boost::geometry::get<1, 0>(segment), boost::geometry::get<1, 1>(segment) };

    // compute the distaces form the starting and ending point and the distance
    // from the segment
    double d = boost::geometry::distance(currentPosition, segment);
    double dp1 = boost::geometry::distance(currentPosition, p1);
    double dp2 = boost::geometry::distance(currentPosition, p2);

    // find the end of the segment nearest to the current position
    // if the distance form the current position to the end of the segment is
    // ugual to d then i am in the middle zone

    if (dp1 < dp2 && dp1 == d) {

        double normAlignVector = std::sqrt(std::pow(currentPosition.y() - p1.y(), 2) + std::pow(currentPosition.x() - p1.x(), 2));

        alignVector(0) = (currentPosition.x() - p1.x()) / normAlignVector;
        alignVector(1) = (currentPosition.y() - p1.y()) / normAlignVector;

        //if the robot is outside the polygon the directionis the opposite
        if (!boost::geometry::covered_by(currentPosition, poly_)) {
            d = -d;
            alignVector(0) = -alignVector(0);
            alignVector(1) = -alignVector(1);
        }
        std::cout << "DEBUG ZONE IN THE MIDDLE: " << std::endl;
    } else if (dp1 > dp2 && dp2 == d) {

        double normAlignVector = std::sqrt(std::pow(currentPosition.y() - p2.y(), 2) + std::pow(currentPosition.x() - p2.x(), 2));

        alignVector(0) = (currentPosition.x() - p2.x()) / normAlignVector;
        alignVector(1) = (currentPosition.y() - p2.y()) / normAlignVector;

        //if the robot is outside the polygon the directionis the opposite
        if (!boost::geometry::covered_by(currentPosition, poly_)) {
            d = -d;
            alignVector(0) = -alignVector(0);
            alignVector(1) = -alignVector(1);
        }

        std::cout << "DEBUG ZONE IN THE MIDDLE: " << std::endl;

    } else {

        // if is inside one of the two area defined by the two segment I will take
        // as alignVector the normal to the segment
        point_type u;
        ComputeNormalVector2Segment(segment, u);
        alignVector(0) = u.x();
        alignVector(1) = u.y();

        if (!boost::geometry::covered_by(currentPosition, poly_)) {
            d = -d;
        }
        std::cout << "DEBUG 1/3 ZONE: " << std::endl;
    }

    SetControlVariable(Eigen::Vector3d{ d, 0.0, 0.0 });

    std::cout << "DEBUG d: " << d << std::endl;
}

void SafetyBoundaries::ComputeAlignVectorConvex(std::list<segment_type> segments, point_type currentPosition, Eigen::Vector3d& alignVector)
{
    //The variable that is continue in the convex case is the distance vector from the inner border of the safety zone
    //As in the concave case we have basically theree situations in which the robot can be:
    //- the robot is near of of the two nearest segments: only one activaction function is active
    //- the robot is the middle zone of the two segments: two activation function are active
    //- the robot is outside the segment

    //Compute the distance from the nearest segment
    double d = boost::geometry::distance(currentPosition, segments.front());

    std::list<point_type> points;

    ComputeIntersectionPointMiddleZone(segments, points);

    point_type intersecP = points.front();
    points.pop_front();

    point_type pMin = points.front();
    points.pop_front();

    point_type pMax = points.front();

    if (currentPosition.x() < pMax.x() && currentPosition.x() > pMin.x() && currentPosition.y() < pMax.y() && currentPosition.y() > pMin.y()) {
        //Find the intersection point in wich the direction od the alignment must be direct in the middle zone of the two segments

        //Compute the align vector if the robot is in the middle zone of two segments
        //In this case the align vector is the direction vector from the current position to the inner point
        alignVector(0) = (currentPosition.x() - intersecP.x());
        alignVector(1) = (currentPosition.y() - intersecP.y());

        alignVector = alignVector.norm() * alignVector;

        //if the robot is outside the polygon the directionis the opposite
        if (!boost::geometry::covered_by(currentPosition, poly_)) {
            d = -d;
            alignVector(0) = -alignVector(0);
            alignVector(1) = -alignVector(1);
        }
        std::cout << "DEBUG ZONE IN THE MIDDLE: " << std::endl;
    } else {
        //Compute the align vector if the robot is near one of the two nearest segments
        //In this case the align vector is the normal to the segment
        point_type u;
        std::cout << "DEBUG 1/3 ZONE: " << std::endl;

        ComputeNormalVector2Segment(segments.front(), u);

        alignVector(0) = u.x();
        alignVector(1) = u.y();

        //if the robot is outside the polygon the directionis the opposite
        if (!boost::geometry::covered_by(currentPosition, poly_)) {
            d = -d;
        }
    }

    SetControlVariable(Eigen::Vector3d{ d, 0.0, 0.0 });

    std::cout << "DEBUG d: " << d << std::endl;
}

bool SafetyBoundaries::ComputeIntersectionPointMiddleZone(std::list<segment_type> segments, std::list<point_type>& points)
{
    // starting point of the first segment
    point_type p1{ boost::geometry::get<0, 0>(segments.front()), boost::geometry::get<0, 1>(segments.front()) };
    // ending point of the segment
    point_type p2{ boost::geometry::get<1, 0>(segments.front()), boost::geometry::get<1, 1>(segments.front()) };
    //starting point of the second segment
    point_type s1{ boost::geometry::get<0, 0>(segments.back()), boost::geometry::get<0, 1>(segments.back()) };
    // ending point of the second segment
    point_type s2{ boost::geometry::get<1, 0>(segments.back()), boost::geometry::get<1, 1>(segments.back()) };

    point_type frontSegDirPerp, backSegDirPerp, frontSegDir, backSegDir;

    ComputeNormalVector2Segment(segments.front(), frontSegDirPerp, frontSegDir);
    ComputeNormalVector2Segment(segments.back(), backSegDirPerp, backSegDir);

    point_type newP1 = { p1.x() + decreasingBellShape_.xmax(0) * frontSegDirPerp.x(), p1.y() + decreasingBellShape_.xmax(0) * frontSegDirPerp.y() };
    point_type newP2 = { p2.x() + decreasingBellShape_.xmax(0) * frontSegDirPerp.x(), p2.y() + decreasingBellShape_.xmax(0) * frontSegDirPerp.y() };
    point_type newS1 = { s1.x() + decreasingBellShape_.xmax(0) * backSegDirPerp.x(), s1.y() + decreasingBellShape_.xmax(0) * backSegDirPerp.y() };
    point_type newS2 = { s2.x() + decreasingBellShape_.xmax(0) * backSegDirPerp.x(), s2.y() + decreasingBellShape_.xmax(0) * backSegDirPerp.y() };

    std::list<segment_type> newSegments;
    MakeSegments(newP1, newP2, newSegments.front());
    MakeSegments(newS1, newS2, newSegments.back());
    std::deque<point_type> out;
    //find the intersection point
    if (!boost::geometry::intersection(newSegments.front(), newSegments.back(), out))
        return -1;

    point_type intersecP;

    intersecP.set<0>((out.front()).x());
    intersecP.set<1>((out.front()).y());

    points.push_back(intersecP);

    //compute the area'plane in which the alignment must be direct to the intersection point
    double k = 50.0; //m
    point_type r1 = { intersecP.x() + k * frontSegDir.x(), intersecP.y() + k * frontSegDir.y() };
    point_type r2 = { intersecP.x() + k * backSegDir.x(), intersecP.y() + k * backSegDir.y() };

    //find the min/max x e y
    double xMin = Eigen::Vector3d{ intersecP.x(), r1.x(), r2.x() }.minCoeff();
    double xMax = Eigen::Vector3d{ intersecP.x(), r1.x(), r2.x() }.maxCoeff();
    double yMin = Eigen::Vector3d{ intersecP.y(), r1.y(), r2.y() }.minCoeff();
    double yMax = Eigen::Vector3d{ intersecP.y(), r1.y(), r2.y() }.maxCoeff();

    point_type pMin = { xMin, yMin };
    point_type pMax = { xMax, yMax };

    points.push_back(pMin);
    points.push_back(pMax);

    return 0;
}

void SafetyBoundaries::ComputeNormalVector2Segment(segment_type segment, point_type& uPerp)
{
    // function to compute the normale vector to a segment that
    // point towards the center of the polygon

    // starting point of the first segment
    point_type p1{ boost::geometry::get<0, 0>(segment), boost::geometry::get<0, 1>(segment) };

    // ending point of the segment
    point_type p2{ boost::geometry::get<1, 0>(segment), boost::geometry::get<1, 1>(segment) };

    point_type u, midP;

    // find the magnitude of the segment
    double p2p1Mag = std::sqrt(std::pow(p2.y() - p1.y(), 2) + std::pow(p2.x() - p1.x(), 2));

    // Direction of the segment
    u.set<0>(1 / p2p1Mag * (p2.x() - p1.x()));
    u.set<1>(1 / p2p1Mag * (p2.y() - p1.y()));

    // find the medium point of the segment
    midP.set<0>((p2.x() + p1.x()) / 2);
    midP.set<1>((p2.y() + p1.y()) / 2);

    // take the direction perpendicular to the segment
    // there are two possible direction orthogonal to the segment. Take the
    // one that point towards the polygon
    uPerp.set<0>(-u.y());
    uPerp.set<1>(u.x());

    //compute the controid of the poly
    point_type centroid;
    boost::geometry::centroid(poly_, centroid);

    point_type direction2Centr;
    // find the magnitude of direction2Centr
    double direction2CentrMag = std::sqrt(std::pow(midP.y() - centroid.y(), 2) + std::pow(midP.x() - centroid.x(), 2));

    // Direction of the segment
    direction2Centr.set<0>(1 / direction2CentrMag * (centroid.x() - midP.x()));
    direction2Centr.set<1>(1 / direction2CentrMag * (centroid.y() - midP.y()));

    // if not take the oder direction
    if (boost::geometry::dot_product(direction2Centr, uPerp) < 0) {
        uPerp.set<0>(u.y());
        uPerp.set<1>(-u.x());
    }
}

void SafetyBoundaries::ComputeNormalVector2Segment(segment_type segment, point_type& uPerp, point_type& u)
{
    // function to compute the normale vector to a segment that
    // point towards the center of the polygon

    // starting point of the first segment
    point_type p1{ boost::geometry::get<0, 0>(segment), boost::geometry::get<0, 1>(segment) };

    // ending point of the segment
    point_type p2{ boost::geometry::get<1, 0>(segment), boost::geometry::get<1, 1>(segment) };

    point_type midP;

    // find the magnitude of the segment
    double p2p1Mag = std::sqrt(std::pow(p2.y() - p1.y(), 2) + std::pow(p2.x() - p1.x(), 2));

    // Direction of the segment
    u.set<0>(1 / p2p1Mag * (p2.x() - p1.x()));
    u.set<1>(1 / p2p1Mag * (p2.y() - p1.y()));

    // find the medium point of the segment
    midP.set<0>((p2.x() + p1.x()) / 2);
    midP.set<1>((p2.y() + p1.y()) / 2);

    // take the direction perpendicular to the segment
    // there are two possible direction orthogonal to the segment. Take the
    // one that point towards the polygon
    uPerp.set<0>(-u.y());
    uPerp.set<1>(u.x());

    //compute the controid of the poly
    point_type centroid;
    boost::geometry::centroid(poly_, centroid);

    point_type direction2Centr;
    // find the magnitude of direction2Centr
    double direction2CentrMag = std::sqrt(std::pow(midP.y() - centroid.y(), 2) + std::pow(midP.x() - centroid.x(), 2));

    // Direction of the segment
    direction2Centr.set<0>(1 / direction2CentrMag * (centroid.x() - midP.x()));
    direction2Centr.set<1>(1 / direction2CentrMag * (centroid.y() - midP.y()));

    // if not take the oder direction
    if (boost::geometry::dot_product(direction2Centr, uPerp) < 0) {
        uPerp.set<0>(u.y());
        uPerp.set<1>(-u.x());
    }
}

void SafetyBoundaries::ExtractMinDistanceSegments(std::list<segment_type> originalSegments, point_type currentPosition, std::list<segment_type>& segments)
{

    // Funtion that comeback the two min distance segment

    // current evalueting distance
    double d = static_cast<double>(INFINITY);
    // current min distance
    double minD = static_cast<double>(INFINITY);

    // list iterators
    std::list<segment_type>::iterator it, tmp;

    it = originalSegments.begin();

    // extract the first min distance segment
    for (std::list<segment_type>::iterator it = originalSegments.begin(); it != originalSegments.end(); it++) {

        // distance from the current potion to the i-th segment
        d = boost::geometry::distance(currentPosition, *it);

        if (d <= minD) {
            minD = d;
            tmp = it;
        }
    }

    segments.push_back(*tmp);
    // erase the the found segment and repeat the same procedure to find the
    // second min dist segment
    originalSegments.erase(tmp);

    d = static_cast<double>(INFINITY);
    minD = static_cast<double>(INFINITY);

    for (std::list<segment_type>::iterator it = originalSegments.begin(); it != originalSegments.end(); it++) {

        // distance from the current potion to the i-th segment
        d = boost::geometry::distance(currentPosition, *it);
        if (d <= minD) {
            minD = d;
            tmp = it;
        }
    }

    segments.push_back(*tmp);
}

bool SafetyBoundaries::IsConvex(std::list<segment_type> segments)
{
    // function to detect if the current side of the polygon is convex or not
    bool isConvex = false;
    // direction vector of the line passing throught the two segments
    point_type midP;

    // starting point of the first segment
    point_type p1{ boost::geometry::get<0, 0>(segments.front()), boost::geometry::get<0, 1>(segments.front()) };
    // ending point of the segment
    point_type p2{ boost::geometry::get<1, 0>(segments.front()), boost::geometry::get<1, 1>(segments.front()) };

    point_type s1{ boost::geometry::get<0, 0>(segments.back()), boost::geometry::get<0, 1>(segments.back()) };

    // ending point of the segment
    point_type s2{ boost::geometry::get<1, 0>(segments.back()), boost::geometry::get<1, 1>(segments.back()) };

    // find the medium point of the segment
    if (p2.x() == s1.x() && p2.y() == s1.y()) {
        midP.set<0>((s2.x() + p1.x()) / 2);
        midP.set<1>((s2.y() + p1.y()) / 2);
    } else {
        midP.set<0>((s1.x() + p2.x()) / 2);
        midP.set<1>((s1.y() + p2.y()) / 2);
    }

    // check if the medium point is inside the polygon. If it is inside then the
    // side of the polygon is convex
    if (boost::geometry::covered_by(midP, poly_))
        isConvex = true;

    return isConvex;
}
void SafetyBoundaries::MakeSegments(point_type const& p, point_type const& next, segment_type& seg)
{
    boost::geometry::set<0, 0>(seg, p.x());
    boost::geometry::set<0, 1>(seg, p.y());
    boost::geometry::set<1, 0>(seg, next.x());
    boost::geometry::set<1, 1>(seg, next.y());
}
} // namespace ikcl
