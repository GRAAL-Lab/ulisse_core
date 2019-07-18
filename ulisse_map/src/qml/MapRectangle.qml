import QtQuick 2.0
import QtLocation 5.9
import QtPositioning 5.9

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

    property Component mapCanvasComponent
    property MapCanvas _canvas

    property var angle: 30
    property var offset: 10

    property var intersections_canvas: []
    property var centroid
    property var intersections_cartesian: []
    property var intersections_geographic: []

    property var px_multiplier


    Component.onCompleted: {
        Helper.init_lib(QtPositioning)
        mapCanvasComponent = Qt.createComponent("MapCanvas.qml");
        _canvas = mapCanvasComponent.createObject(map)
        map.addMapItem(_canvas)
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
                rect_phase = 0
                mapMouseArea.hoverEnabled = false
                line.color = "#33cc33"
                generate_path()
                draw_path()
                generate_nurbs()
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
                var lam = Helper.lat_to_m_coeff(coordinateAt(0).latitude)
                var lom = Helper.lon_to_m_coeff(coordinateAt(0).longitude)
                p1 = Helper.project(p0,m,p1, lam/lom)
            }
            replaceCoordinate(1, map.toCoordinate(p1))
        } else if (rect_phase === 2){
            var cp0 = coordinateAt(0);
            var cp1 = coordinateAt(1);
            var cpc = map.toCoordinate(Qt.point(mouse.x, mouse.y), false)
            var coords = [cp0, cp1, cpc]
            var centroid = Helper.coords_centroid(coords)
            var lam = Helper.lat_to_m_coeff(centroid.latitude)
            var lom = Helper.lon_to_m_coeff(centroid.longitude)
            var points = Helper.points_map2euclidean(coords, centroid, lam, lom)

            var m = Helper.slope(points[0], points[1])
            var m_perp = -Math.pow(lam/lom,2)/m //perpendicularity constraint for aspect ratios different than 1:1
            var p2 = Helper.intersect_two_lines(points[1], m_perp, points[2], m)
            var p3 = Helper.intersect_two_lines(points[0], m_perp, points[2], m)
            var cp2 = Helper.point_euclidean2map(p2.x, p2.y, centroid, lam, lom)
            var cp3 = Helper.point_euclidean2map(p3.x, p3.y, centroid, lam, lom)

            if (cp2 !== null && cp2.isValid && cp3 !== null && cp3.isValid){
                replaceCoordinate(2, cp2)
                replaceCoordinate(3, cp3)
            }
        }
    }

    function generate_path(){
        var orig_tilt = map.tilt
        map.tilt = 0

        var shape_coords = path.slice(0, pathLength()-1)

        centroid = Helper.coords_centroid(shape_coords)
        var limits = Helper.shape_geo_limits(shape_coords)

        var dim = Helper.shape_px_dimensions(limits, map)

        var lam = Helper.lat_to_m_coeff(centroid.latitude)
        var lom = Helper.lon_to_m_coeff(centroid.longitude)

        // transform shape coordinates and work in a virtual euclidean metric system
        var points = Helper.points_map2euclidean(shape_coords, centroid, lam, lom)
        points = Helper.set_points_clockwise(points)
        var sides = Helper.make_sides(points)

        intersections_cartesian = Helper.convex_polygon_parallel_slices_intersections(angle, offset, sides, lam/lom)
        intersections_cartesian = Helper.rectify_dense_winding(intersections_cartesian, lam/lom)


        // transform virtual euclidean reference points in map coordinates
        intersections_geographic = Helper.segments_euclidean2map(intersections_cartesian, centroid, lam, lom)

        var a = map.toCoordinate(Qt.point(0,0))
        var b = map.toCoordinate(Qt.point(0,1))
        var c = map.toCoordinate(Qt.point(1,0))
        var p2m_h = a.distanceTo(b)
        var p2m_v = a.distanceTo(c)

        offset *= 3
        var o_x = offset/p2m_h
        var o_y = offset/p2m_v

        Helper.init_canvas(_canvas, map,
                           dim[0]+2*o_x, dim[1]+2*o_y,
                           limits.max_lat + offset/lam, limits.min_lon - offset/lom,
                           Helper.relative_zoom_pixel_ratio(map, map.maximumZoomLevel))

        offset /= 3

        // transform map coordinates in canvas pixel indexes
        intersections_canvas = Helper.segments_map2canvas(intersections_geographic, map, _canvas)
        map.tilt = orig_tilt
    }

    function to_debug_canvas(pt, limits, lam, lom, canvas){
        var scale = 1
        return Qt.point(canvas.canvasSize.width-(canvas.canvasSize.width/2.0 + pt.x/scale), canvas.canvasSize.height/2.0 + pt.y/scale)
    }

    function draw_path(){
        var lam = Helper.lat_to_m_coeff(centroid.latitude)
        var lom = Helper.lon_to_m_coeff(centroid.longitude)

        // clear the canvas
        _canvas.canvasCtx.clearRect(0, 0, _canvas.canvasWidth, _canvas.canvasHeight)


        // draw parallel lines
        Helper.draw_path_lines(_canvas, intersections_canvas)

        // draw manouvre curves
        var a = 0, b = 1
        for (var i=0; i<intersections_canvas.length-1; i++){
            a = Helper.flip(a)
            b = Helper.flip(b)
            var p0 = intersections_canvas[i][b]
            var p1 = intersections_canvas[i][a]
            var p2 = intersections_canvas[i+1][a]
            var p3 = intersections_canvas[i+1][b]
            var l1 = Helper.distance(p0, p1)
            var l2 = Helper.distance(p3, p2)
            var d = Helper.distance(p1, p2)
            var c = 0.55191502449 * d/2
            var pc1a = Helper.interpolate(p0, p1, l1+c, 1)
            var pc1b = Helper.interpolate(p3, p2, l2+c, 1)
            var _a = Helper.interpolate(p0, p1, l1+d/2, 1)
            var _b = Helper.interpolate(p3, p2, l2+d/2, 1)
            var pc3 = Helper.interpolate(_a, _b, d/2, 1)
            var pc2a = Helper.interpolate(pc3, _a, c, 1)
            var pc2b = Helper.interpolate(pc3, _b, c, 1)
            Helper.draw_cubic_bezier(_canvas.canvasCtx, p1, pc1a, pc2a, pc3)
            Helper.draw_cubic_bezier(_canvas.canvasCtx, pc3, pc2b, pc1b, p2)
        }
    }

    function generate_nurbs(){
        var nurb_l = []
        var nurb_c = []
        for (var i=0; i<intersections_cartesian.length; i++){
            var p0 = intersections_cartesian[i][0]
            var p1 = intersections_cartesian[i][1]
            nurb_l.push(Helper.generate_nurb_line(p0, p1))
        }

        for (var i=0; i<intersections_cartesian.length-1; i++){
            var dir = (i+1)%2
            var p0 = intersections_cartesian[i][dir]
            var p3 = intersections_cartesian[i+1][dir]
            nurb_l.push(Helper.generate_nurb_circle(p0, p3, dir))
        }

        var curves = [nurb_l[0]]
        for (var i=0; i<intersections_cartesian.length-1; i++){
            curves.push(nurb_l[i])
            curves.push(nurb_l[i+1])
        }
        var result = {
            centroid: [centroid.latitude, centroid.longitude],
            curves: curves
        }
        //console.log(JSON.stringify(result))
        return result
    }
}
