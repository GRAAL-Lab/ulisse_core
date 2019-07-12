import QtQuick 2.0
import QtLocation 5.6
import QtPositioning 5.6

import "../scripts/helper.js" as Helper

MapPolyline {
    line.width: 3
    line.color: "#81c784"
    opacity: 1.0
    id: root
    z: map.z + 5

    signal end

    property var rect_phase: 0
    property var w
    property var h

    function max_x(){
        var max = map.fromCoordinate(coordinateAt(0)).x
        for (var i=1; i<pathLength(); i++){
            var x = map.fromCoordinate(coordinateAt(i)).x
            if (x>max) max=x
        }
        return max
    }

    function max_y(){
        var max = map.fromCoordinate(coordinateAt(0)).y
        for (var i=1; i<pathLength(); i++){
            var y = map.fromCoordinate(coordinateAt(i)).y
            if (y>max) max=y
        }
        return max
    }

    function min_x(){
        var min = map.fromCoordinate(coordinateAt(0)).x
        for (var i=1; i<pathLength(); i++){
            var x = map.fromCoordinate(coordinateAt(i)).x
            if (x<min) min=x
        }
        return min
    }

    function min_y(){
        var min = map.fromCoordinate(coordinateAt(0)).y
        for (var i=1; i<pathLength(); i++){
            var y = map.fromCoordinate(coordinateAt(i)).y
            if (y<min) min=y
        }
        return min
    }

    function coords_centroid(coords){
        var lat = 0
        var lon = 0
        for (var i=0; i<coords.length; i++){
            lat += coords[i].latitude
            lon += coords[i].longitude
        }
        return QtPositioning.coordinate(lat/coords.length, lon/coords.length)
    }

    function click_handler(mouse){
        if (mouse.button & Qt.LeftButton){
            if (rect_phase === 0){
                var p = Qt.point(mouse.x, mouse.y)
                var wp = map.toCoordinate(p)
                addCoordinate(wp)
                addCoordinate(wp)
                mapMouseArea.hoverEnabled = true
                rect_phase = 1
            } else if (rect_phase === 1){
                var p = Qt.point(mouse.x, mouse.y)
                var wp = map.toCoordinate(p)
                var wp0 = coordinateAt(0)
                addCoordinate(wp)
                addCoordinate(wp0)
                addCoordinate(wp0)
                rect_phase = 2
                line.color = "#ffb300"
            } else if (rect_phase === 2){
                var p0 = map.fromCoordinate(coordinateAt(0))
                var p2 = map.fromCoordinate(coordinateAt(2))
                h = max_y() - min_y()
                w = max_x() - min_x()
                rect_phase = 0
                mapMouseArea.hoverEnabled = false
                line.color = "#33cc33"
                draw()
                end()
            }
        }
    }

    function pos_changed_handler(mouse){
        if (rect_phase === 1){
            var p0 = map.fromCoordinate(coordinateAt(0));
            var p1 = Qt.point(mouse.x, mouse.y)
            if (mouse.modifiers & Qt.ShiftModifier){
                var n_intervals = 16
                var snap_interval = 2*Math.PI/n_intervals
                var theta = Math.atan2(p1.y-p0.y, p1.x-p0.x)
                var snap_idx = Math.floor((theta + snap_interval/2) / snap_interval)
                var snap_theta = snap_idx*snap_interval
                var m = Math.tan(snap_theta)
                p1 = Helper.project(p0,m,p1)
            }
            replaceCoordinate(1, map.toCoordinate(p1))
        } else if (rect_phase === 2){
            var p0 = map.fromCoordinate(coordinateAt(0));
            var p1 = map.fromCoordinate(coordinateAt(1));
            var m = Helper.slope(p1, p0)
            var m_perp = -1/m
            var pm = Qt.point(mouse.x, mouse.y)
            var p2 = Helper.project(p1, m_perp, pm)
            var p3 = Helper.project(p0, m_perp, pm)
            var cp2 = map.toCoordinate(p2, true)
            var cp3 = map.toCoordinate(p3, true)
            if (cp2.isValid && cp3.isValid){
                replaceCoordinate(2, cp2)
                replaceCoordinate(3, cp3)
            }
        }
    }

    property Component mapCanvasComponent
    property MapCanvas _canvas

    Component.onCompleted: {
        mapCanvasComponent = Qt.createComponent("MapCanvas.qml");
        _canvas = mapCanvasComponent.createObject(map)
        map.addMapItem(_canvas)
    }

    property var angle: 30
    property var offset: 5


    function side_angle(side){
        var a = Helper.rad_to_deg(Math.atan(Helper.slope(side[0], side[1])))
        return (a>=0) ? a : a+90
    }

    function intersections(point, angle, sides){
        var l0 = Helper.to_homogeneous_line(point, Helper.deg_to_rad(angle))

        var ii = []
        for (var i=0; i<sides.length; i++){
            var s = sides[i]
            var li = Helper.to_homogeneous_line(s[0], Math.atan(Helper.slope(s[0], s[1])))

            var intersection = l0.crossProduct(li)
            if (intersection.z !== 0){/*not parallel*/
                var pi = Helper.from_homogeneous_point(intersection)
                if (Helper.point_in_box(pi, s[0], s[1]))
                    ii.push(pi)
            }
        }
        return ii
    }


    function set_points_clockwise(points){
        if (Helper.three_point_direction(points[0],points[1],points[2]) === 1)
            points.reverse()
        return points
    }

    function find_interesting_features(points){
        var t = Helper.find_top(points)
        var pti = t[1]
        var pt = t[0]
        var pl = points[pti > 0 ? pti-1 : points.length-1]

        var sides = []
        for (var i=0, idx_a=pti-1, idx_b; i<points.length; i++){
            idx_a = Helper.add_and_wrap(idx_a, points.length)
            idx_b = Helper.add_and_wrap(idx_a, points.length)
            sides.push([points[idx_a], points[idx_b]])
        }
        var upper_side = sides[sides.length-1]
        return {
            point_top: pt,
            point_before: pl,
            upper_side: upper_side,
            sides: sides
        }
    }

    function map_to_euclidean(){
        var centroid = coords_centroid(path.slice(0,4))
        var lam = Helper.lat_to_m_coeff(centroid.latitude)
        var lom = Helper.lon_to_m_coeff(centroid.longitude)
        var points = []
        for (var i = 0; i<4; i++){
            points.push(Qt.point((coordinateAt(i).longitude - centroid.longitude) * -lom,
                               (coordinateAt(i).latitude  - centroid.latitude)  * -lam))
        }
        return {
            centroid: centroid,
            points: points,
            lam: lam,
            lom: lom
        }
    }

    function map_to_canvas(cc, map, canvas){
        var pp = []
        for (var i=0; i<cc.length; i++){
            var a = Helper.toCanvasCoordinates(fromCoordinate(cc[i][0]), map, canvas)
            var b = Helper.toCanvasCoordinates(fromCoordinate(cc[i][1]), map, canvas)
            pp.push([a,b])
        }
        return pp
    }

    function euclidean_to_map(ii, centroid, lam, lom){
        var cc = []
        for (var i=0; i<ii.length; i++){
            cc.push([QtPositioning.coordinate(centroid.latitude + ii[i][0].y/-lam, centroid.longitude + ii[i][0].x/-lom),
                     QtPositioning.coordinate(centroid.latitude + ii[i][1].y/-lam, centroid.longitude + ii[i][1].x/-lom)])
        }
        return cc
    }

    function init_canvas(canvas, map, width, height, coordinate){
        canvas.canvasWidth = width
        canvas.canvasHeight = height
        canvas.zoomLevel = map.zoomLevel
        canvas.coordinate = coordinate
        canvas.canvasAngle = map.bearing
    }

    function draw_line(ctx,x1,y1,x2,y2){
        ctx.beginPath()
        ctx.moveTo(x1,y1)
        ctx.lineTo(x2,y2)
        ctx.closePath()
        ctx.stroke()
    }

    function draw_bounding_rect(canvas, w, h){
        var ctx = canvas.canvasCtx
        var o = map.fromCoordinate(canvas.coordinate)
        ctx.fillStyle = Qt.rgba(0, 0.4, 0, 0.2)
        ctx.fillRect(0, 0, w, h)
        canvas.requestPaint()
    }

    function find_intersections(angle, pt, pl, upper_side, sides){
        var au_deg = 90-angle
        var at_deg = side_angle(upper_side)
        var offset_on_t = offset/Math.cos(Helper.deg_to_rad(90-au_deg))

        var times=0
        var ii=[]
        while (true){
            ++times
            var p = Helper.interpolate(pt, pl, offset_on_t, times)
            var _i = intersections(p, at_deg-au_deg, sides) //NB two intersections always if a convex polygon
            if (_i.length < 2) break
            if (_i[0].x < _i[1].x)
                ii.push([_i[0], _i[1]])
            else
                ii.push([_i[1], _i[0]])
        }
        return ii
    }

    function rectify_dense_winding(ii){
        var a = 0, b = 1
        for (var i = 0; i < ii.length-1; i++){
            var rr = Helper.rectify_parallelogram_side(ii[i][a], ii[i][b], ii[i+1][b], ii[i+1][a])
            ii[i][b] = rr[1]
            ii[i+1][b] = rr[2]
            a = Helper.flip(a)
            b = Helper.flip(b)
        }
        return ii
    }

    function draw_path_lines(canvas, pp){
        var ctx = canvas.canvasCtx
        for (var i = 0; i < pp.length; i++){
            var p0 = pp[i][0]
            var p1 = pp[i][1]
            draw_line(ctx, p0.x, p0.y, p1.x, p1.y)
            canvas.requestPaint()
        }
    }

    function draw_path_semicircles(canvas, pp){
        var ctx = canvas.canvasCtx
        for (var i = 0; i < pp.length-1; i++){
            var dir = (i+1)%2
            var p0 = pp[i][dir]
            var p1 = pp[i+1][dir]
            ctx.beginPath()
            ctx.moveTo(p1.x, p1.y)
            ctx.arc((p0.x+p1.x)/2,
                    (p0.y+p1.y)/2,
                    Helper.distance(p0, p1)/2,
                    Math.atan2(p1.y-p0.y, p1.x-p0.x),
                    Math.atan2(p1.y-p0.y, p1.x-p0.x) + Math.PI,
                    dir === 1)
            ctx.moveTo(p1.x, p1.y)
            ctx.closePath()
            ctx.stroke()
        }
        canvas.requestPaint()
    }

    function draw(){

        // transform shape coordinates and work in a virtual euclidean metric system
        var r1 = map_to_euclidean()
        var centroid = r1.centroid
        var points = set_points_clockwise(r1.points)
        var lam = r1.lam
        var lom = r1.lom

        var r2 = find_interesting_features(points)
        var pt = r2.point_top
        var pl = r2.point_before
        var upper_side = r2.upper_side
        var sides = r2.sides

        var intersections_cartesian = find_intersections(angle, pt, pl, upper_side, sides)
        intersections_cartesian = rectify_dense_winding(intersections_cartesian)

        // transform virtual euclidean reference points in map coordinates
        var intersections_geographic = euclidean_to_map(intersections_cartesian, centroid, lam, lom)

        var a = map.toCoordinate(Qt.point(0,0))
        var b = map.toCoordinate(Qt.point(0,100))
        var p2m = a.distanceTo(b)/100.0

        var offset_px = offset/p2m
        init_canvas(_canvas, map,
                    w+2*offset_px, h+2*offset_px,
                    map.toCoordinate(Qt.point(min_x()-offset_px, min_y()-offset_px)))

        // transform map coordinates in canvas pixel indexes
        var intersections_canvas = map_to_canvas(intersections_geographic, map, _canvas)

        // draw a bounding box around the area
        draw_bounding_rect(_canvas, w+2*offset_px, h+2*offset_px)

        // draw parallel lines
        draw_path_lines(_canvas, intersections_canvas)

        // draw manouvre curves
        draw_path_semicircles(_canvas, intersections_canvas)

        generate_nurbs(centroid, intersections_cartesian)
    }

    function nurbs_line(p0, p1){
        return {
            dim: 1,
            ctrl_pts: [
                [p0.x, p0.y, 0, 1],
                [p1.x, p1.y, 0, 1]
            ],
            knots: [

            ]
        }
    }

    function generate_nurbs(centroid, intersections_p){
        var nurb_l = []
        var nurb_c = []
        for (var i=0; i<intersections_p.length; i++){
            var p0 = intersections_p[i][0]
            var p1 = intersections_p[i][1]
            nurb_l.push({
                degree: 1,
                points: [[p0.x, p0.y], [p1.x, p1.y]],
                weigths: [1, 1],
                knots: [0, 0, 1, 1]
            })
            console.log(Helper.serialize({
                "discretization": 100,
                "degree": 1,
                "controlPoints": [p0.x, p0.y, 0, 1, p1.x, p1.y, 0, 1],
                "knots": [0, 0, 1, 1]
            }, 10))
        }

        for (var i=0; i<intersections_p.length-1; i++){
            var dir = (i+1)%2
            var p0 = intersections_p[i][dir]
            var p3 = intersections_p[i+1][dir]
            var dist = Helper.distance(p0, p3)
            var angle = Math.tan(1/-Helper.slope(p0, p3))
            if (dir === 1){
                var p1 = Qt.point(p0.x + Math.cos(angle)*dist, p0.y + Math.sin(angle)*dist)
                var p2 = Qt.point(p3.x + Math.cos(angle)*dist, p3.y + Math.sin(angle)*dist)
            } else {
                var p1 = Qt.point(p0.x - Math.cos(angle)*dist, p0.y - Math.sin(angle)*dist)
                var p2 = Qt.point(p3.x - Math.cos(angle)*dist, p3.y - Math.sin(angle)*dist)
            }
            nurb_l.push({
                degree: 3,
                points: [[p0.x, p0.y], [p1.x, p1.y], [p2.x, p2.y], [p3.x, p3.y]],
                weigths: [1, 1/3.0, 1/3.0, 1],
                knots: [0, 0, 0, 0, 1, 1, 1, 1]
            })
            console.log(Helper.serialize({
                "discretization": 100,
                "degree": 3,
                "controlPoints": [p0.x, p0.y, 0, 1, p1.x, p1.y, 0, 1/3.0, p2.x, p2.y, 0, 1/3.0, p3.x, p3.y, 0, 1],
                "knots": [0, 0, 0, 0, 1, 1, 1, 1]
            }, 10))
        }
        console.log(nurb_c)
    }
}
