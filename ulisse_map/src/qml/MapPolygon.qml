import QtQuick 2.0
import QtLocation 5.9
import QtPositioning 5.9

import "../scripts/helper.js" as Helper


MapPolyline {
    id: root
    line.width: 3
    opacity: 1.0
    z: map.z + 5

    signal end

    property var polygonal_phase: 0
    property var poligonal_direction: 0 //1: counterclockwise, 2: clockwise


    property Component mapCanvasComponent
    property Component mapMarkerComponent
    property Component mapDashedLineComponent
    property Component mapHandleComponent
    property MapDashedLine _dashed_line
    property MapCanvas _canvas
    property MapMarker _marker
    property MapHandle _handle
    property var vertex_markers: []
    property var add_markers: []

    property real _angle: 30
    property real _offset: 10
    property real jump: 2
    property string method: "single_winding" //"simple"

    property var debug_c: null

    property var intersections_canvas: []
    property var centroid
    property var intersections_cartesian: []
    property var intersections_geographic: []

    property var px_multiplier

    Component.onCompleted: {
        Helper.init_lib(QtPositioning)
        mapCanvasComponent = Qt.createComponent("MapCanvas.qml");
        mapMarkerComponent = Qt.createComponent("MapMarker.qml");
        mapDashedLineComponent = Qt.createComponent("MapDashedLine.qml");
        mapHandleComponent = Qt.createComponent("MapHandle.qml");

        _canvas = mapCanvasComponent.createObject(map)
        _marker = mapMarkerComponent.createObject(map)
        _dashed_line = mapDashedLineComponent.createObject(map)
        _handle = mapHandleComponent.createObject(map)

        map.addMapItem(_canvas)
        map.addMapItem(_marker)
        map.addMapItem(_dashed_line)
        map.addMapItem(_handle)
        _handle.add_to_map(map)
    }

    function deregister_map_items(){
        map.removeMapItem(_canvas)
        map.removeMapItem(_marker)
        map.removeMapItem(_dashed_line)
        map.removeMapItem(_handle)
        _handle.deregister_map_items(map)
    }

    function _draw(){
        generate_path()
        draw_path()
        generate_nurbs()
        generate_markers()
        disable_markers()
    }

    function draw_deferred(){
        if (_canvas.canvasCtx !== null && _canvas.canvasCtx !== undefined)
            _draw()
        else
            _canvas.canvasCtxChanged.connect(_draw)
    }

    function get_path(){
        return path.slice(0,path.length-1)
    }

    function get_next_in_path(pos, plus){
        var _path = get_path()
        return _path[Helper.add_and_wrap(pos,_path.length, plus)]
    }

    function get_prev_in_path(pos, minus){
        var _path = get_path()
        return _path[Helper.sub_and_wrap(pos, _path.length, minus)]
    }

    function generate_markers(){
        var _path = path.slice(0,path.length-1)
        for (var i = 0; i< _path.length; i++){
            var marker1 = mapMarkerComponent.createObject(map)
            var marker2 = mapMarkerComponent.createObject(map)
            map.addMapItem(marker1)
            map.addMapItem(marker2)
            vertex_markers.push(marker1)
            add_markers.push(marker2)
        }
        reposition_vertex_markers()
        reposition_add_markers()
        reposition_handle()
        disable_handle()
        update_scale()
    }

    function reposition_handle(){
        var c = map.fromCoordinate(centroid, false)
        _handle.h_center = centroid
        _handle.h_handle = map.toCoordinate(Qt.point(c.x-40, c.y), false)
        _handle.cumulativeAngle=0
        _handle.update_canvas(0)
    }

    function reposition_vertex_markers(){
        var _path = get_path()
        for (var i = 0; i< vertex_markers.length; i++){
            vertex_markers[i].center = _path[i]
            vertex_markers[i].opacity = 0
            vertex_markers[i].color = "#00ff00"
        }
    }

    function reposition_add_markers(){
        var _path = get_path()
        for (var i = 0; i< add_markers.length; i++){
            add_markers[i].center = Helper.geo_midpoint(_path[i], get_next_in_path(i,1))
            add_markers[i].opacity = 0
            add_markers[i].color = "#bbbb00"
        }
    }

    function reposition_markers(){
        reposition_add_markers()
        reposition_vertex_markers()
    }

    function enable_handle(){
        _handle.opacity = 1
    }

    function disable_handle(){
        _handle.cumulativeAngle=0
        _handle.update_canvas(0)
        _handle.opacity = 0
    }

    function enable_markers(){
        enable_vertex_markers()
        enable_add_markers()
    }

    function enable_vertex_markers(){
        for (var i = 0; i< vertex_markers.length; i++)
            vertex_markers[i].opacity = 1
    }

    function enable_add_markers(){
        for (var i = 0; i< add_markers.length; i++)
            add_markers[i].opacity = 1
    }

    function disable_markers(){
        disable_add_markers()
        disable_vertex_markers()
    }

    function disable_vertex_markers(){
        for (var i = 0; i< vertex_markers.length; i++)
            vertex_markers[i].opacity = 0
    }

    function disable_add_markers(){
        for (var i = 0; i< add_markers.length; i++)
            add_markers[i].opacity = 0
    }

    function clear_path(){
        for(var i=0; i<path.length; i++)
            removeCoordinate(0)
    }

    function check_safe(box){
        var bpp = [], ppp = []
        for(var i=0; i<box.path.length; i++)
            bpp.push(map.fromCoordinate(box.path[i],false))
        for(var i=0; i<path.length; i++)
            ppp.push(map.fromCoordinate(path[i],false))
        safe = Helper.polygons_disjoint(bpp,ppp) && Helper.coord_inside_polygon(path[0], box.path)
        return safe
    }

    line.color: "#81c784"
    property var safe: null
    onSafeChanged: function(){
        line.color = (safe ? "#00ff00" : "#ff0000")
    }

    //FIXME: check it in the euclidean plane
    function map_polygon_point_admissibility(pc){
        var pa = map.fromCoordinate(path[pathLength()-3])
        var pb = map.fromCoordinate(path[pathLength()-2])
        var pd = map.fromCoordinate(path[0])
        var pe = map.fromCoordinate(path[1])
        return (poligonal_direction === Helper.three_point_direction(pa,pb,pc)
        &&  poligonal_direction === Helper.three_point_direction(pb,pc,pd)
        &&  poligonal_direction === Helper.three_point_direction(pc,pd,pe))
    }

    function map_polygon_point_mod_admissibility(pc, idx){
        var _path = get_path()
        var pa = map.fromCoordinate(get_next_in_path(idx,2))
        var pb = map.fromCoordinate(get_next_in_path(idx,1))
        var pd = map.fromCoordinate(get_prev_in_path(idx,1))
        var pe = map.fromCoordinate(get_prev_in_path(idx,2))
        var r1 = Helper.three_point_direction(pa,pb,pc)
        var r2 = Helper.three_point_direction(pb,pc,pd)
        if (r1 !== r2) return false
        var r3 = Helper.three_point_direction(pc,pd,pe)
        if (r2 !== r3) return false
        return true
    }

    function click_handler_rect(mouse){
        if (mouse.button & Qt.LeftButton){
            if (polygonal_phase === 0){
                var p = Qt.point(mouse.x, mouse.y)
                var wp = map.toCoordinate(p)
                addCoordinate(wp)
                addCoordinate(wp)
                mapMouseArea.hoverEnabled = true
                polygonal_phase = 1
            } else if (polygonal_phase === 1){
                var p = Qt.point(mouse.x, mouse.y)
                var wp = map.toCoordinate(p)
                var wp0 = coordinateAt(0)
                addCoordinate(wp)
                addCoordinate(wp0)
                addCoordinate(wp0)
                polygonal_phase = 2
                line.color = "#ffb300"
            } else if (polygonal_phase === 2){
                var p0 = map.fromCoordinate(coordinateAt(0))
                var p2 = map.fromCoordinate(coordinateAt(2))
                polygonal_phase = 0
                mapMouseArea.hoverEnabled = false
                line.color = "#33cc33"
                generate_path()
                draw_path()
                generate_nurbs()
                end()
            }
        }
    }

    function pos_changed_handler_rect(mouse){
        if (polygonal_phase === 1){
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
        } else if (polygonal_phase === 2){
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



    function pos_changed_handler(mouse, box){
        if (polygonal_phase === 0) return;
        var p = Qt.point(mouse.x, mouse.y)
        var color
        var pf = map.toCoordinate(p)
        if (polygonal_phase === 3 && !map_polygon_point_admissibility(p)){
            pf = path[0]
            color = "#ffb300"
        } else {
            color = "#81c784"
        }
        line.color = color
        replaceCoordinate(path.length-1, pf)
    }

    function click_handler(mouse){
        var m = Qt.point(mouse.x, mouse.y)
        var p = map.toCoordinate(m)
        if (mouse.button & Qt.LeftButton){
            if (polygonal_phase === 0){
                clear_path()
                addCoordinate(p)
                addCoordinate(p)
                polygonal_phase = 1
                mapMouseArea.hoverEnabled = true
            } else if (polygonal_phase === 1){
                addCoordinate(p)
                polygonal_phase = 2
            } else if (polygonal_phase === 2){
                var p1 = map.fromCoordinate(path[0])
                var p2 = map.fromCoordinate(path[1])
                poligonal_direction = Helper.three_point_direction(p1,p2,m)
                addCoordinate(p)
                polygonal_phase = 3
            } else if (polygonal_phase === 3){
                if (map_polygon_point_admissibility(m))
                    addCoordinate(p)
                else close_polygon()
            }
        } else if (mouse.button & Qt.RightButton){
            close_polygon()
        }
    }

    property var moving_idx: -1
    property var marked: -1
    property bool translating: false
    property bool rotating: false

    onTranslatingChanged: function(){
        reposition_markers()
        reposition_handle()
        return (translating||rotating) ? disable_markers() : enable_markers()
    }

    onRotatingChanged: function(){
        reposition_markers()
        reposition_handle()
        return (translating||rotating) ? disable_markers() : enable_markers()
    }

    function nearest_marker(point, markers, nearest, thresh){
        for (var i=0; i<markers.length; i++){
            var v = map.fromCoordinate(markers[i].center)
            var d = Helper.distance(point,v)
            if(d < thresh){
                thresh = d
                nearest = i
            }
        }
        return {nearest:nearest, distance:thresh}
    }

    function nearest_than(p, d1, n1, p2, n2){
        var d2 = Helper.distance(p,p2)
        return (d1<d2) ? {nearest:n1, distance:d1}:{nearest:n2, distance:d2}
    }

    function change_marked(nearest){
        var _path = get_path()
        if (marked>=0 && marked < _path.length){
            vertex_markers[marked].color = "#00ff00"
            _dashed_line.reset()
        }
        else if(marked >= _path.length && marked < 2*_path.length){
            add_markers[marked-_path.length].color = "#bbbb00"
        }
        marked = nearest
        if (marked>=0 && marked < _path.length){
            vertex_markers[marked].color = "#ff0000"
            if (_path.length > 3){
                _dashed_line.replaceCoordinate(0, get_next_in_path(marked,1))
                _dashed_line.replaceCoordinate(1, get_prev_in_path(marked,1))
            }
        } else if(marked >= _path.length && marked < 2*_path.length){
            add_markers[marked-_path.length].color = "#ff0000"
        }
        _handle.center_select(marked === 2*_path.length)
        _handle.handle_select(marked === 2*_path.length+1)
    }

    function change_coordinate(idx, coord){
        replaceCoordinate(idx, coord)
        vertex_markers[idx].center = coord
        if (idx === 0)
            replaceCoordinate(path.length-1, coord)
        update_centroid()
    }

    function remove_coordinate(idx){
        removeCoordinate(idx)
        if (idx === 0)
            replaceCoordinate(pathLength()-1, path[0])
        map.removeMapItem(add_markers[idx])
        map.removeMapItem(vertex_markers[idx])
        add_markers.splice(idx, 1)
        vertex_markers.splice(idx, 1)
        reposition_add_markers()
        update_centroid()
    }

    function add_mid_coordinate(_idx){
        var idx = _idx+1
        add_markers[_idx].color = "#bbbb00"
        var c = add_markers[_idx].center
        insertCoordinate(idx, c)
        var marker1 = mapMarkerComponent.createObject(map)
        map.addMapItem(marker1)
        marker1.center = c
        marker1.opacity = 1
        marker1.color = "#00ff00"
        vertex_markers.splice(idx,0,marker1)
        var marker2 = mapMarkerComponent.createObject(map)
        map.addMapItem(marker2)
        marker2.center = c
        marker2.opacity = 0
        marker2.color = "#bbbb00"
        add_markers.splice(idx,0,marker2)
    }

    function move_coordinates(center){
        for (var i = 0; i<path.length; i++){
            var c = path[i]
            var cn = QtPositioning.coordinate(c.latitude-centroid.latitude+center.latitude,
                                              c.longitude-centroid.longitude+center.longitude)
            replaceCoordinate(i,cn)
        }
        update_centroid()
    }

    function rotate_and_scale_coordinates(angle, scale){
        for (var i = 0; i<path.length; i++){
            var c = path[i]
            var d = centroid.distanceTo(c)
            var a = centroid.azimuthTo(c)
            var cn = centroid.atDistanceAndAzimuth(d*scale, a+angle)
            replaceCoordinate(i, cn)
        }
        _handle.update_canvas(angle)
    }

    function pos_changed_mod_handler(mouse){
        var p = Qt.point(mouse.x, mouse.y)
        var pf = map.toCoordinate(p)
        var _path = get_path()
        if (moving_idx === -1 && !translating && !rotating){
            var hpc = map.fromCoordinate(_handle.h_center)
            var hph = map.fromCoordinate(_handle.h_handle)
            var r1 = nearest_marker(p, add_markers, -1, 7)
            var r2 = nearest_marker(p, vertex_markers, (r1.nearest === -1) ? -1 : r1.nearest + _path.length, r1.distance)
            var r3 = nearest_than(p, r2.distance, (r2.nearest === -1) ? -1 : r2.nearest, hpc, 2*_path.length)
            var r4 = nearest_than(p, r3.distance, (r3.nearest === -1) ? -1 : r3.nearest, hph, 2*_path.length+1)
            change_marked(r4.nearest)
        } else if (moving_idx>=0 && moving_idx<_path.length){
            if (map_polygon_point_mod_admissibility(p, moving_idx)){
                color = "#81c784"
            } else {
                color = "#ffb300"
                pf = path[moving_idx]
            }
            line.color = color
            change_coordinate(moving_idx, pf)
        } else if (translating){
            move_coordinates(map.toCoordinate(p))
        } else if (rotating){
            var scale = _handle.h_center.distanceTo(pf)/_handle.h_center.distanceTo(_handle.h_handle)
            var angle = _handle.h_center.azimuthTo(pf)-_handle.h_center.azimuthTo(_handle.h_handle)
            rotate_and_scale_coordinates(angle, scale)
            _handle.h_handle = pf
        }
    }

    function update_centroid(){
        var _path = path.slice(0,path.length-1)
        centroid = Helper.coords_centroid(_path)
        var c_p = map.fromCoordinate(centroid)
        _handle.h_center = centroid
        _handle.h_handle = map.toCoordinate(Qt.point(c_p.x-40, c_p.y), false)
    }

    function update_scale(){
        var a = map.toCoordinate(Qt.point(0,0))
        var b = map.toCoordinate(Qt.point(0,1))
        var p2m = a.distanceTo(b)
        var r = 10*p2m
        for (var i = 0; i< vertex_markers.length; i++)
            vertex_markers[i].radius = r
        for (var i = 0; i< add_markers.length; i++)
            add_markers[i].radius = r
        _handle.h_radius = r
    }

    function click_mod_handler(mouse){
        mapMouseArea.hoverEnabled = false
        _dashed_line.reset()
        var p = Qt.point(mouse.x, mouse.y)
        var pf = map.toCoordinate(p)
        var _path = get_path()
        if (moving_idx === -1){
            var hpc = map.fromCoordinate(_handle.h_center)
            var hph = map.fromCoordinate(_handle.h_handle)
            var r1 = nearest_marker(p, add_markers, -1, 7)
            var r2 = nearest_marker(p, vertex_markers, (r1.nearest === -1) ? -1 : r1.nearest + _path.length, r1.distance)
            var r3 = nearest_than(p, r2.distance, (r2.nearest === -1) ? -1 : r2.nearest, hpc, 2*_path.length)
            var r4 = nearest_than(p, r3.distance, (r3.nearest === -1) ? -1 : r3.nearest, hph, 2*_path.length+1)
            var nearest = r4.nearest
            if (mouse.button & Qt.LeftButton){
                if(nearest >=0 && nearest < _path.length){
                    disable_add_markers()
                    disable_handle()
                    moving_idx = nearest
                } else if (nearest >= _path.length && nearest < 2*_path.length){
                    add_mid_coordinate(nearest-_path.length)
                    enable_markers()
                    disable_add_markers()
                    disable_handle()
                    moving_idx = nearest-_path.length + 1
                } else if (nearest === 2*_path.length){
                    translating = !translating
                } else if (nearest === 2*_path.length+1){
                    rotating = !rotating
                }
            } else if (mouse.button & Qt.RightButton){
                if (nearest >=0 && nearest < _path.length){
                    if (_path.length > 3){
                        remove_coordinate(nearest)
                        enable_add_markers()
                    }
                }
            }
        } else if (moving_idx >= 0){
            moving_idx = -1
            reposition_add_markers()
            enable_add_markers()
            enable_handle()
        }
        mapMouseArea.hoverEnabled = true
    }


    function close_polygon(){
        if (polygonal_phase === 3){
            replaceCoordinate(pathLength()-1, path[0])
            mapMouseArea.hoverEnabled = false
            line.color = "#33cc33"
            polygonal_phase = 0
            generate_path()
            draw_path()
            generate_nurbs()
            generate_markers()
            disable_markers()
            disable_handle()
            end()
        }
    }

    function generate_path(){
        if (!(method === "simple" || method === "single_winding")) return

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

        intersections_cartesian = Helper.convex_polygon_parallel_slices_intersections(_angle, _offset, sides, lam/lom)

        if (method === "simple" || method === "single_winding")
            intersections_cartesian = Helper.rectify_dense_winding(intersections_cartesian, lam/lom)

        // transform virtual euclidean reference points in map coordinates
        intersections_geographic = Helper.segments_euclidean2map(intersections_cartesian, centroid, lam, lom)

        var a = map.toCoordinate(Qt.point(0,0))
        var b = map.toCoordinate(Qt.point(0,1))
        var c = map.toCoordinate(Qt.point(1,0))
        var p2m_h = a.distanceTo(b)
        var p2m_v = a.distanceTo(c)

        _offset *= 3
        var o_x = _offset/p2m_h
        var o_y = _offset/p2m_v

        Helper.init_canvas(_canvas, map,
                           dim[0]+2*o_x, dim[1]+2*o_y,
                           limits.max_lat + _offset/lam, limits.min_lon - _offset/lom,
                           Helper.relative_zoom_pixel_ratio(map, map.maximumZoomLevel))

        _offset /= 3

        // transform map coordinates in canvas pixel indexes
        intersections_canvas = Helper.segments_map2canvas(intersections_geographic, map, _canvas)
        map.tilt = orig_tilt
    }

    function to_debug_canvas(pt, limits, lam, lom, canvas){
        var scale = 1
        return Qt.point(canvas.canvasSize.width-(canvas.canvasSize.width/2.0 + pt.x/scale), canvas.canvasSize.height/2.0 + pt.y/scale)
    }

    property var backup_path
    property var backup_vertex_markers
    property var backup_add_markers
    property var backup_centroid

    function begin_edit(){
        backup_path=path
        moving_idx = -1
        backup_path = path
        backup_centroid = centroid
        backup_vertex_markers = vertex_markers
        backup_add_markers = add_markers
        _canvas.clear_canvas()
        enable_markers()
        enable_handle()
    }

    function discard_edit(){
        moving_idx = -1
        translating = false
        rotating = false
        path=backup_path
        vertex_markers = backup_vertex_markers
        add_markers = backup_add_markers
        centroid = backup_centroid
        disable_markers()
        disable_handle()
        generate_path()
        draw_path()
        generate_nurbs()
    }

    function confirm_edit(angle, offset){
        _angle=angle
        _offset=offset
        moving_idx = -1
        disable_markers()
        disable_handle()
        generate_path()
        draw_path()
        generate_nurbs()
    }

    function draw_path(){
        // clear the canvas
        _canvas.clear_canvas()

        // draw parallel lines
        Helper.draw_path_lines(_canvas, intersections_canvas)

        // draw manouvre curves
        if (method === "simple")
           Helper.draw_manouvre_simple(_canvas, intersections_canvas)
        else if (method === "single_winding")
           Helper.draw_manouvre_single_winding(_canvas, intersections_canvas)
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
            if (method === "simple")
                nurb_l.push(Helper.generate_nurb_line(p0, p3))
            else if (method === "single_winding")
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

    function serialize(){
        var values = []
        for (var j = 0; j < path.length; j++){
            var p_i = path[j]
            values.push({
                latitude: p_i.latitude,
                longitude: p_i.longitude
            })
        }
        return {
            name: 'PolyPath',
            offset: _offset,
            angle: _angle,
            values: values
        }

    }

    function deserialize(values){
        var lat, lon
        for(var j = 0; j < values.length; j++){
            lat = values[j].latitude
            lon = values[j].longitude
            poly_cur.addCoordinate(QtPositioning.coordinate(lat,lon))
        }
    }
}
