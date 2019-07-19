#include <iomanip>

#include "rclcpp/rclcpp.hpp"

#include "ulisse_ctrl/ctrl_data_structs.hpp"
#include "ulisse_ctrl/fsm_defines.hpp"

#include "rml/RML.h"

/*
#include "../include/tinynurbs/tinynurbs.h"
#include "glm/glm.hpp"
#include "glm/gtc/constants.hpp"
 */
#include <cmath>

#include <openNURBS/opennurbs.h>

using namespace ulisse;

/*
static int compar_dbl(const double* a, const double* b)
{
    if ( *a < *b )
        return -1;
    if ( *a > *b )
        return 1;
    return 0;
}

bool ON_PolyCurve::GetClosestPoint( const ON_3dPoint& test_point,
                                    double* t,       // parameter of local closest point returned here
                                    double maximum_distance,
                                    const ON_Interval* sub_domain
) const
{
    ON_Workspace ws;
    const int count = Count();
    bool rc = false;
    if ( count > 0 ) {
        const ON_Curve* curve;
        ON_BoundingBox bbox;
        int i, *index;
        double d, t0, t1, s0, s1, s;
        ON_SimpleArray<double> near_dist(count);
        ON_Interval seg_domain, crv_domain;

        std::cout << "ok" << std::endl;
        // get curve domain to search
        GetDomain(&t0,&t1);
        if ( sub_domain ) {
            s0 = sub_domain->Min();
            s1 = sub_domain->Max();
            if ( s0 > t0 )
                t0 = s0;
            if ( s1 < t1 )
                t1 = s1;         // GBA 12/17/02   fixed bug   was   s1 = t1;
        }
        if ( t0 > t1 )
            return false; // nothing to search
        if ( t0 == t1 ) {
            if ( maximum_distance > 0.0 ) {
                d = test_point.DistanceTo(PointAt(t0));
                if ( d <= maximum_distance )
                    rc = true;
            }
            else
                rc = true;
            if ( rc && t )
                *t = t0;
            return rc;
        }

        // get minimum distance from test_point to each segment's bounding box
        for ( i = 0; i < count; i++ ) {
            d = 1.0e300;
            curve = m_segment[i];
            if ( m_t[i] <= t1 && m_t[i+1] >= t0 && curve ) {
                bbox = curve->BoundingBox();
                if ( bbox.IsValid() )
                    d = test_point.DistanceTo(bbox.ClosestPoint(test_point));
            }
            near_dist.Append(d);
        }

        // sort segments so closest segments are tested first
        index = ws.GetIntMemory(count);
        near_dist.Sort( ON::heap_sort, index, compar_dbl );

        // test each segment
        if ( maximum_distance <= 0.0 )
            maximum_distance = -1.0;
        for ( i = 0; i < count; i++ ) {
            if ( maximum_distance > 0.0 && near_dist[index[i]] > maximum_distance )
                break; // every untested segment is too far away to matter

            if ( near_dist[index[i]] >= 1.0e300 )
                continue; // segment skipped for some reason

            curve = m_segment[index[i]];

            // get sub_domain for this curve
            crv_domain = curve->Domain();
            seg_domain.Set(m_t[index[i]],m_t[index[i]+1]);
            s0 = seg_domain.NormalizedParameterAt(t0);
            s1 = seg_domain.NormalizedParameterAt(t1);
            if ( s0 < 0.0 )
                s0 = 0.0;
            if ( s1 > 1.0)
                s1 = 1.0;
            if ( s0 > 0.0 || s1 < 1.0 ) {
                sub_domain = &seg_domain;
                s0 = crv_domain.ParameterAt(s0);
                s1 = crv_domain.ParameterAt(s1);
                crv_domain.Set(s0,s1);
                sub_domain = &crv_domain;
            }
            else {
                sub_domain = 0;
            }

            // test this curve
            if ( curve->GetClosestPoint( test_point, &s, maximum_distance, sub_domain ) ) {
                std::cout << "ok" << std::endl;
                d = test_point.DistanceTo(curve->PointAt(s));
                if ( maximum_distance < 0.0 || d < maximum_distance ) {
                    // this point is the best one we've got so far
                    maximum_distance = d;
                    if ( t ){
                        // note that crv_domain is changed above so we need to
                        // go back to the original to get the right interval.
                        crv_domain = curve->Domain();
                        if ( crv_domain == seg_domain )
                        {
                            // 22 September Dale Lear - keep answer more accurate
                            //     in common case when seg_domain = segment curve domain.
                            *t = s;
                        }
                        else
                        {
                            // Apply an affine map to adjust the parameter value
                            double np = crv_domain.NormalizedParameterAt(s);
                            *t = seg_domain.ParameterAt(np);
                        }
                    }
                    std::cout << "ok" << std::endl;
                    rc = true;
                    if ( d == 0.0 )
                        break; // can't do any better than this
                }
            }
        }
    }
    return rc;
}
*/

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("nurbs");

    int rate = 10;
    rclcpp::WallRate loop_rate(rate);

    /*
    ON_BOOL32 bIsRational = true;
    ON_NurbsCurve crv(2, bIsRational, 4, 4);

    bool b = false;

    ON_3dPoint p1 = ON_3dPoint(1.0, 0.0, 0.0);
    ON_3dPoint p2 = ON_3dPoint(1.0, 2.0, 0.0);
    ON_3dPoint p3 = ON_3dPoint(-1.0, 2.0, 0.0);
    ON_3dPoint p4 = ON_3dPoint(-1.0, 0.0, 0.0);

    b = crv.SetCV(0, p1);
    std::cout << "Result : " << b << std::endl;
    b = crv.SetCV(1, p2);
    std::cout << "Result : " << b << std::endl;
    b = crv.SetCV(2, p3);
    std::cout << "Result : " << b << std::endl;
    b = crv.SetCV(3, p4);
    std::cout << "Result : " << b << std::endl;

    b = crv.SetWeight(0, 1.0);
    std::cout << "Result : " << b << std::endl;
    b = crv.SetWeight(1, 0.33);
    std::cout << "Result : " << b << std::endl;
    b = crv.SetWeight(2, 0.33);
    std::cout << "Result : " << b << std::endl;
    b = crv.SetWeight(3, 1.0);
    std::cout << "Result : " << b << std::endl;

    b = crv.SetKnot(0, 0.0);
    std::cout << "Result : " << b << std::endl;
    b = crv.SetKnot(1, 0.0);
    std::cout << "Result : " << b << std::endl;
    b = crv.SetKnot(2, 0.0);
    std::cout << "Result : " << b << std::endl;
    b = crv.SetKnot(3, 1.0);
    std::cout << "Result : " << b << std::endl;
    b = crv.SetKnot(4, 1.0);
    std::cout << "Result : " << b << std::endl;
    b = crv.SetKnot(5, 1.0);
    std::cout << "Result : " << b << std::endl;

    std::cout << "Degree : " << crv.Degree() << std::endl;

    if (crv.IsValid()){

        std::cout << "DIO CANEEEEEE" << std::endl;
    }

    ON_3dPoint PPP = ON_3dPoint(1.0, 0.0, 0.0);
    double ascissa = 1000.0;
    b = crv.GetClosestPoint(PPP, &ascissa, 1000.0);
    std::cout << "Result : " << b << std::endl;
    std::cout << "Closest Point: " << ascissa << std::endl;
    std::cout << "POINT : " << PPP.x << ", " << PPP.y << " , " << PPP.z << std::endl;

    double length = 201.0;
    b = crv.GetLength(&length);
    std::cout << "Result : " << b << std::endl;
    std::cout << "GetLength: " << length << std::endl;


    ON_3dPoint P = crv.PointAt(1.0);
    std::cout << "POINT : " << P.x << ", " << P.y << " , " << P.z << std::endl;
    P = crv.PointAt(0.0);
    std::cout << "POINT : " << P.x << ", " << P.y << " , " << P.z << std::endl;

    /*

    std::cout << "P1 : " << p1.x << ", " << p1.y << " , " << p1.z << std::endl;

    ON_SimpleArray<ON_3dPoint> points(4);
    points.SetCount(4);
    points[0] = ON_3dPoint(1.0, 0.0, 1.0);
    points[1] = ON_3dPoint(1.0, 2.0, 1.0);
    points[2] = ON_3dPoint(-1.0, 2.0, 1.0);
    points[3] = ON_3dPoint(-1.0, 0.0, 1.0);

    ON_SimpleArray<double> weights(4);
    weights.SetCount(4);
    weights[0] = 1.0;
    weights[1] = 0.33;
    weights[2] = 0.33;
    weights[3] = 1.0;

    /*
    ON_SimpleArray<double> knots(8);
    knots.SetCount(8);
    knots[0] = 0.000;
    knots[1] = 0.000;
    knots[2] = 0.000;
    knots[3] = 0.000;
    knots[4] = 1.000;
    knots[5] = 1.000;
    knots[6] = 1.000;
    knots[7] = 1.000;

    ON_NurbsCurve curve(3, bIsRational, 4, 4);

    for (int ci = 0; ci < curve.CVCount(); ci++)
    {
        ON_4dPoint p4d(points[ci].x * weights[ci], points[ci].y * weights[ci], points[ci].z * weights[ci], weights[ci]);
        curve.SetCV(ci, p4d);
    }

    for (int ki = 0; ki < 8; ki++)
        curve.m_knot[ki] = knots[ki];

    if (curve.IsValid()){

        std::cout << "***** CONVERGE *****" << std::endl;
    }
     */

    bool b;
    int dimension = 3;
    ON_BOOL32 bIsRational = true;
    int degree = 3;
    int cv_count = 4;
    int knot_count = cv_count + degree - 1;

    ON_SimpleArray<ON_3dPoint> points(cv_count);
    points.SetCount(cv_count);
    points[0] = ON_3dPoint(1.000, 0.000, 0.000);
    points[1] = ON_3dPoint(1.000, 2.000, 0.000);
    points[2] = ON_3dPoint(-1.000, 2.000, 0.000);
    points[3] = ON_3dPoint(-1.000, 0.000, 0.000);

    ON_SimpleArray<double> weights(cv_count);
    weights.SetCount(cv_count);
    weights[0] = 1.0;
    weights[1] = 0.4;
    weights[2] = 0.4;
    weights[3] = 1.0;

    ON_SimpleArray<double> knots(knot_count);
    knots.SetCount(knot_count);
    knots[0] = 0.000;
    knots[1] = 0.000;
    knots[2] = 0.000;
    knots[3] = 1.000;
    knots[4] = 1.000;
    knots[5] = 1.000;

    ON_NurbsCurve nc(dimension, bIsRational, degree + 1, cv_count);
    nc.Create(dimension, bIsRational, degree + 1, cv_count);
    nc.ReserveCVCapacity( cv_count );
    nc.ReserveKnotCapacity( degree+cv_count-1 );
    nc.MakeRational();


    for (int ci = 0; ci < nc.CVCount(); ci++)
    {
        nc.SetCV(ci, points[ci]);
        nc.SetWeight(ci, weights[ci]);
    }

    for (int ki = 0; ki < knot_count; ki++)
        nc.m_knot[ki] = knots[ki];


    nc.SetDomain(0.0, 1.0);

    if (nc.IsValid())
    {
        std::cout << "***** CONVERGE *****" << std::endl;

        b = nc.SetStartPoint(points[0]);
        std::cout << "Result Set Start: " << b << std::endl;

        ON_3dPoint PPP = ON_3dPoint(1.0, 1.0, 1.0);
        double ascissa = 0.0;
        //b = nc.GetClosestPoint(PPP, &ascissa, 1000.0, &interval);
        b = nc.GetClosestPoint(PPP, &ascissa);
        std::cout << "Result : " << b << std::endl;
        std::cout << "Closest Point: " << ascissa << std::endl;
        std::cout << "POINT : " << PPP.x << ", " << PPP.y << " , " << PPP.z << std::endl;

        //ON_Line line(points[0],points[1]);
        //b=line.ClosestPointTo( PPP, &ascissa );

        /*
        ON_PolyCurve pc(1);
        pc.Append (&nc);

        b = pc.GetClosestPoint(PPP, &ascissa);

        std::cout << "Result : " << b << std::endl;
        std::cout << "Closest Point: " << ascissa << std::endl;
        std::cout << "POINT : " << PPP.x << ", " << PPP.y << " , " << PPP.z << std::endl;

         */

        /*
        double length = 0.0;
        b = nc.GetLength(&length,  1.0e-8, &interval);
        std::cout << "Result : " << b << std::endl;
        std::cout << "GetLength: " << length << std::endl;

         */

        /*
        ON_3dPoint P = nc.PointAt(1.0);
        std::cout << "POINT at 1: " << P.x << ", " << P.y << " , " << P.z << std::endl;
        P = nc.PointAt(0.0);
        std::cout << "POINT at 0 : " << P.x << ", " << P.y << " , " << P.z << std::endl;


        ON_3dVector v = nc.TangentAt(0.0);
        std::cout << "TANGENT at 0 : " << v.x << ", " << v.y << " , " << v.z << std::endl;

        P = nc.PointAt(0.5);
        std::cout << "POINT at 0.5 : " << P.x << ", " << P.y << " , " << P.z << std::endl;
        v = nc.TangentAt(0.5);
        std::cout << "TANGENT at 0.5 : " << v.x << ", " << v.y << " , " << v.z << std::endl;
         */
    }

    /*
    tinynurbs::RationalCurve<2, float> crv; // Planar curve using float32
    crv.control_points = {glm::vec2(1, 0), // std::vector of 2D points
                          glm::vec2(1, 2),
                          glm::vec2(-1, 2),
                          glm::vec2(-1, 0)
    };;
    crv.weights = {1, 1/3, 1/3, 1}; // std::vector of floats,
    crv.knots = {0, 0, 0, 0, 1, 1, 1, 1}; // std::vector of floats
    crv.degree = 3;

    glm::vec2 pt = tinynurbs::curvePoint(crv, 0.f);
    // Outputs a point [-1, 0]
    glm::vec2 tgt = tinynurbs::curveTangent(crv, 0.5f);
    // Outputs a vector [1, 0]

    std::cout << "Curve Point: (" << pt.x << " , " << pt.y << " )" << std::endl;

    std::cout << "Curve Point: (" << tgt.x << " , " << tgt.y << " )" << std::endl;
*/

    /*
    using namespace PLib ;
    int deg = 3 ;

    //using namespace PLib ;

    Vector_HPoint3Df P(10) ;

    P[0] = HPoint3Df(70,50,0,1) ;
    P[1] = HPoint3Df(100,60,0,1) ;
    P[2] = HPoint3Df(110,120,0,1) ;
    P[3] = HPoint3Df(100,150,0,1) ;
    P[4] = HPoint3Df(180,150,0,1) ;
    P[5] = HPoint3Df(200,200,0,1) ;
    P[6] = HPoint3Df(80,190,0,1) ;
    P[7] = HPoint3Df(60,100,0,1) ;
    P[8] = HPoint3Df(50,70,0,1) ;
    P[9] = P[0] ;

    PlNurbsCurvef curve1 ;
    PlNurbsCurvef curve2 ;

    curve1.globalInterpH(P,deg) ;

    cout << "U1= " << curve1.knot() << endl ;
    cout << "P1 = " << curve1.ctrlPnts() << endl ;
    cout << "D1 = " << curve1.degree() << endl ;

    P.resize(9); // the closed loop routine doesn't need P[0] = P[last]

    Vector_HPoint3Df Pw ;
    wrapPointVectorH(P,deg,Pw);
    curve2.globalInterpClosedH(Pw,deg) ;


    cout << "U2 = " << curve2.knot() << endl ;
    cout << "P2 = " << curve2.ctrlPnts() << endl ;
    cout << "D2 = " << curve2.degree() << endl ;

     */


    return 0;
}

