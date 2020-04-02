#include "ulisse_ctrl/tasks/SafetyBoundaries.h"
#include "ulisse_ctrl/helper_functions.hpp"

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
    x_dot_ = taskParameter_.gain * desiredVelocity_;
}

void SafetyBoundaries::UpdateInternalActivationFunction()
{
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

    }

    // check if the robot is near the two segment concav
    else {
        // compute the direct of alignent to escape from the border in case of
        // concave side of the polygon
        ComputeAlignVector(minDistsegments.front(), currentPosition, alignVector);
    }
}

void SafetyBoundaries::ComputeAlignVector(segment_type segment, point_type currentPosition, Eigen::Vector3d& alignVector)
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
        std::cout << "DEBUG:: ZONE3 : " << d << std::endl;
        alignVector(0) = currentPosition.x() - p1.x();
        alignVector(1) = currentPosition.y() - p1.y();

    } else if (dp1 > dp2 && dp1 == d) {
        std::cout << "DEBUG:: ZONE3: " << d << std::endl;
        alignVector(0) = currentPosition.x() - p2.x();
        alignVector(1) = currentPosition.y() - p2.y();

    } else {
        std::cout << "DEBUG:: ZONE1/2 : " << d << std::endl;
        // if is inside one of the two area defined by the two segment I will take
        // as alignVector the normal to the segment
        ComputeNormalVector2Segment(segment, alignVector);
    }

    // if the robot is inside the polygon, the align vector is in the opposite
    // direct of the one computed, which is from the current posiiton to a point
    // on the border
    if (boost::geometry::covered_by(currentPosition, poly_)) {
        std::cout << "DEBUG:: IAM INSIDE THE POLYGON" << std::endl;

    } else {
        std::cout << "DEBUG:: IAM outside THE POLYGON" << std::endl;
        d = -d;
    }
    std::cout << "DEBUG:: d : " << d << std::endl;
    // compute the activation function of the if is distance under threshold
    Ai_(0, 0) = rml::DecreasingBellShapedFunction(decreasingBellShape_.xmin(0), decreasingBellShape_.xmax(0), 0.0, 1.0, d);
}

void SafetyBoundaries::ComputeNormalVector2Segment(segment_type segment, Eigen::Vector3d& alignVector)
{
    // function to compute the normale vector to a segment that
    // point towards the center of the polygon

    // starting point of the first segment
    point_type p1{ boost::geometry::get<0, 0>(segment), boost::geometry::get<0, 1>(segment) };

    // ending point of the segment
    point_type p2{ boost::geometry::get<1, 0>(segment), boost::geometry::get<1, 1>(segment) };

    point_type u, uPerp, midP;

    // find the magnitude of the segment
    double p2p1Mag = std::sqrt(std::pow(p2.y() - p1.y(), 2) + std::pow(p2.x() - p1.x(), 2));

    // Direction of the segment
    u.set<0>(1 / p2p1Mag * (p2.x() - p1.x()));
    u.set<1>(1 / p2p1Mag * (p2.y() - p1.y()));

    std::cout << "Debug u prima prima: " << u.x() << " " << u.y() << std::endl;

    // find the medium point of the segment
    midP.set<0>((p2.x() + p1.x()) / 2);
    midP.set<1>((p2.y() + p1.y()) / 2);

    // take the direction perpendicular to the segment
    // there are two possible direction orthogonal to the segment. Take the
    // one that point towards the polygon
    uPerp.set<0>(-u.y());
    uPerp.set<1>(u.x());

    std::cout << "Debug u prima: " << uPerp.x() << " " << uPerp.y() << std::endl;

    //compute the controid of the poly
    point_type centroid;
    boost::geometry::centroid(poly_, centroid);

    point_type direction2Centr;
    // find the magnitude of direction2Centr
    double direction2CentrMag = std::sqrt(std::pow(midP.y() - centroid.y(), 2) + std::pow(midP.x() - centroid.x(), 2));

    // Direction of the segment
    direction2Centr.set<0>(1 / direction2CentrMag * (centroid.x() - midP.x()));
    direction2Centr.set<1>(1 / direction2CentrMag * (centroid.y() - midP.y()));

    std::cout << "Debug direction centert: " << direction2Centr.x() << " " << direction2Centr.y() << std::endl;

    // if not take the oder direction
    if (boost::geometry::dot_product(direction2Centr, uPerp) < 0) {
        uPerp.set<0>(u.y());
        uPerp.set<1>(-u.x());

        std::cout << "Debug u dopo: " << uPerp.x() << " " << uPerp.y() << std::endl;
    }

    alignVector(0) = uPerp.x();
    alignVector(1) = uPerp.y();
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
