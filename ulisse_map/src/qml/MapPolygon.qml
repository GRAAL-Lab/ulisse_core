import QtQuick 2.0
import QtLocation 5.9
import QtPositioning 5.9

import "../scripts/helper.js" as Helper


MapPolyline {
    line.width: 3
    line.color: "#81c784"
    opacity: 1.0
    z: map.z + 5

    signal end

    property var polygonal_phase: 0
    property var poligonal_direction: 0 //1: counterclockwise, 2: clockwise


    property Component mapCanvasComponent
    property MapCanvas _canvas

    property real angle: 30
    property real offset: 10
    property var debug_c: null


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

    //FIXME: check it in the euclidean plane
    function map_polygon_point_admissibility(pc){
        var pa = map.fromCoordinate(coordinateAt(pathLength()-3))
        var pb = map.fromCoordinate(coordinateAt(pathLength()-2))
        var pd = map.fromCoordinate(coordinateAt(0))
        var pe = map.fromCoordinate(coordinateAt(1))
        return (poligonal_direction === Helper.three_point_direction(pa,pb,pc)
        &&  poligonal_direction === Helper.three_point_direction(pb,pc,pd)
        &&  poligonal_direction === Helper.three_point_direction(pc,pd,pe))
    }


    function pos_changed_handler(mouse){
        if (polygonal_phase === 0) return;
        var p = Qt.point(mouse.x, mouse.y)
        var pf = map.toCoordinate(Qt.point(mouse.x, mouse.y))
        var last_idx = pathLength()-1
        var color = "#81c784"
        if (polygonal_phase === 3 && !map_polygon_point_admissibility(p)){
            pf = coordinateAt(0)
            color = "#ffb300"
        }
        line.color = color
        replaceCoordinate(last_idx, pf)
    }

    function click_handler(mouse){
        if (mouse.button & Qt.LeftButton){
            var m = Qt.point(mouse.x, mouse.y)
            var p = map.toCoordinate(m)
            if (polygonal_phase === 0){
                var l = pathLength()
                for(var i=0; i<l; i++)
                    removeCoordinate(0)
                polygonal_phase = 1
                addCoordinate(p)
                addCoordinate(p)
                mapMouseArea.hoverEnabled = true
            } else if (polygonal_phase === 1){
                polygonal_phase = 2
                addCoordinate(p)
            } else if (polygonal_phase === 2){
                var p1 = map.fromCoordinate(coordinateAt(0))
                var p2 = map.fromCoordinate(coordinateAt(1))
                var p3 = m
                poligonal_direction = Helper.three_point_direction(p1,p2,p3)
                polygonal_phase = 3
                addCoordinate(p)
            } else if (polygonal_phase === 3){
                if (map_polygon_point_admissibility(m))
                    addCoordinate(p)
                else close_polygon()
            }
        } else if (mouse.button & Qt.RightButton){
            close_polygon()
        }
    }

    function close_polygon(){
        if (polygonal_phase === 3){
            replaceCoordinate(pathLength()-1, coordinateAt(0))
            mapMouseArea.hoverEnabled = false
            line.color = "#33cc33"
            polygonal_phase = 0
            generate_path()
            draw_path()
            generate_nurbs()
            end()
        }
    }

    property var points
    property var __steps
    property var limits
    function generate_path(){
        var orig_tilt = map.tilt
        map.tilt = 0

        var shape_coords = path.slice(0, pathLength()-1)

        centroid = Helper.coords_centroid(shape_coords)
        limits = Helper.shape_geo_limits(shape_coords)

        var dim = Helper.shape_px_dimensions(limits, map)

        var lam = Helper.lat_to_m_coeff(centroid.latitude)
        var lom = Helper.lon_to_m_coeff(centroid.longitude)

        // transform shape coordinates and work in a virtual euclidean metric system
        points = Helper.points_map2euclidean(shape_coords, centroid, lam, lom)
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
