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

    property Component mapCanvasComponent
    property MapCanvas _canvas

    property var angle: 30
    property var offset: 5

    property var intersections_canvas: []
    property var centroid_cartesian
    property var intersections_cartesian: []
    property var intersections_geographic: []

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
                h = max_y() - min_y()
                w = max_x() - min_x()
                rect_phase = 0
                mapMouseArea.hoverEnabled = false
                line.color = "#33cc33"
                generate_path()
                draw_path()
                //map.zoomLevelChanged.connect(draw_path)
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

    property var px_multiplier

    function generate_path(){
        // transform shape coordinates and work in a virtual euclidean metric system
        var r1 = Helper.map_to_euclidean(path.slice(0,4))
        centroid_cartesian = r1.centroid
        var points = Helper.set_points_clockwise(r1.points)
        var lam = r1.lam
        var lom = r1.lom

        var r2 = Helper.find_interesting_features(points)
        var pt = r2.point_top
        var pl = r2.point_before
        var upper_side = r2.upper_side
        var sides = r2.sides

        intersections_cartesian = Helper.find_intersections(angle, pt, pl, upper_side, sides, offset)
        intersections_cartesian = Helper.rectify_dense_winding(intersections_cartesian)

        // transform virtual euclidean reference points in map coordinates
        intersections_geographic = Helper.euclidean_to_map(intersections_cartesian, centroid_cartesian, lam, lom)

        var a = map.toCoordinate(Qt.point(0,0))
        var b = map.toCoordinate(Qt.point(0,100))
        var p2m = a.distanceTo(b)/100.0

        var z = map.zoomLevel
        map.zoomLevel = map.maximumZoomLevel
        var c0 = map.toCoordinate(Qt.point(1,1))
        var c1 = map.toCoordinate(Qt.point(2,1))
        map.zoomLevel = z
        var p0 = map.fromCoordinate(c0)
        var p1 = map.fromCoordinate(c1)
        px_multiplier = 1.0/(p1.x-p0.x)

        var offset_px = offset/p2m
        Helper.init_canvas(_canvas, map,
                           (w+2*offset_px), (h+2*offset_px),
                           map.toCoordinate(Qt.point(min_x()-offset_px, min_y()-offset_px)),
                           px_multiplier)

        // transform map coordinates in canvas pixel indexes
        intersections_canvas = Helper.map_to_canvas(intersections_geographic, map, _canvas)
    }

    function draw_path(){
        // clear the canvas
        _canvas.canvasCtx.clearRect(0, 0, _canvas.canvasWidth, _canvas.canvasHeight)

        // draw a bounding box around the area
        Helper.draw_bounding_rect(map, _canvas, _canvas.canvasWidth, _canvas.canvasHeight)

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

}

