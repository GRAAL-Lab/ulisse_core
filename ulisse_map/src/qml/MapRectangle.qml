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
    property var centroid_cartesian
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
                p1 = Helper.project(p0,m,p1)

            }
            replaceCoordinate(1, map.toCoordinate(p1))
        } else if (rect_phase === 2){
            var cp0 = coordinateAt(0);
            var cp1 = coordinateAt(1);
            var a1 = Helper.path_bearing(cp1, cp0)
            var a2 = a1+90

            var cpc = map.toCoordinate(Qt.point(mouse.x, mouse.y), false)

            var cp2 = Helper.nearest_geo_intersection(cp1, a2, cpc, a1)
            var cp3 = Helper.nearest_geo_intersection(cp0, a2, cpc, a1)

            if (cp2 !== null && cp2.isValid && cp3 !== null && cp3.isValid){
                replaceCoordinate(2, cp2)
                replaceCoordinate(3, cp3)
            }
        }
    }

    function generate_path(){
        var orig_tilt = map.tilt
        map.tilt = 0

        // transform shape coordinates and work in a virtual euclidean metric system
        var shape_coords = path.slice(0, pathLength()-1)

        var centroid = Helper.coords_centroid(shape_coords)
        var limits = Helper.shape_geo_limits(shape_coords)

        var dim = Helper.shape_px_dimensions(limits, map)

        var lam = Helper.lat_to_m_coeff(centroid.latitude)
        var lom = Helper.lon_to_m_coeff(centroid.longitude)

        var points = Helper.map_to_euclidean(shape_coords, centroid, lam, lom)
        points = Helper.set_points_clockwise(points)

        var r2 = Helper.find_interesting_features(points)
        var pt = r2.point_top
        var pl = r2.point_before
        var upper_side = r2.upper_side
        var sides = r2.sides

        intersections_cartesian = Helper.find_intersections(angle, pt, pl, upper_side, sides, offset)
        intersections_cartesian = Helper.rectify_dense_winding(intersections_cartesian)

        // transform virtual euclidean reference points in map coordinates
        intersections_geographic = Helper.euclidean_to_map(intersections_cartesian, centroid, lam, lom)

        var a = map.toCoordinate(Qt.point(0,0))
        var b = map.toCoordinate(Qt.point(0,1))
        var c = map.toCoordinate(Qt.point(1,0))
        var p2m_h = a.distanceTo(b)
        var p2m_v = a.distanceTo(c)

        var o_x = offset/p2m_h
        var o_y = offset/p2m_v

        Helper.init_canvas(_canvas, map,
                           dim[0]+2*o_x, dim[1]+2*o_y,
                           limits.max_lat + offset/lam, limits.min_lon - offset/lom,
                           Helper.relative_zoom_pixel_ratio(map, map.maximumZoomLevel))

        // transform map coordinates in canvas pixel indexes
        intersections_canvas = Helper.map_to_canvas(intersections_geographic, map, _canvas)
        map.tilt = orig_tilt
    }

    function draw_path(){
        // clear the canvas
        _canvas.canvasCtx.clearRect(0, 0, _canvas.canvasWidth, _canvas.canvasHeight)

        // draw a bounding box around the area
        //Helper.draw_bounding_rect(map, _canvas, _canvas.canvasWidth, _canvas.canvasHeight)

        // draw parallel lines
        Helper.draw_path_lines(_canvas, intersections_canvas)

        // draw manouvre curves
        Helper.draw_path_semicircles(_canvas, intersections_canvas)
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

        var result = [nurb_l[0]]
        for (var i=0; i<intersections_cartesian.length-1; i++){
            result.push(nurb_l[i])
            result.push(nurb_l[i+1])
        }
        return result
    }
}

