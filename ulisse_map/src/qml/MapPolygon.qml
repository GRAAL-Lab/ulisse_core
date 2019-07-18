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
    property Component mapMarkerComponent
    property Component mapDashedLineComponent
    property Component mapHandleComponent
    property MapDashedLine _dashed_line
    property MapCanvas _canvas
    property MapMarker _marker
    property MapHandle _handle
    property var vertex_markers: []
    property var add_markers: []

    property real angle: 30
    property real offset: 10
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
        map.addMapItem(_canvas)
        _marker = mapMarkerComponent.createObject(map)
        map.addMapItem(_marker)
        _dashed_line = mapDashedLineComponent.createObject(map)
        map.addMapItem(_dashed_line)
        _dashed_line.addCoordinate(QtPositioning.coordinate(0,0))
        _dashed_line.addCoordinate(QtPositioning.coordinate(0,0))
        _handle = mapHandleComponent.createObject(map)
        map.addMapItem(_handle)
        _handle.add_to_map(map)
    }

    function middle_coord(c1, c2){
        return QtPositioning.coordinate(c1.latitude - c2.latitude,
                                        c1.longitude - c2.longitude)
    }

    function generate_markers(){
        var _path = path.slice(0,path.length-1)
        for (var i = 0; i< _path.length; i++){
            var marker1 = mapMarkerComponent.createObject(map)
            map.addMapItem(marker1)
            marker1.center = _path[i]
            marker1.opacity = 0
            marker1.color = "#00ff00"
            vertex_markers.push(marker1)
            var marker2 = mapMarkerComponent.createObject(map)
            map.addMapItem(marker2)
            marker2.center = Helper.geo_midpoint(_path[i],
                                                 _path[Helper.add_and_wrap(i, _path.length)])
            marker2.opacity = 0
            marker2.color = "#bbbb00"
            add_markers.push(marker2)
            _handle.h_center = centroid
            var c = map.fromCoordinate(centroid, false)
            _handle.h_handle = map.toCoordinate(Qt.point(c.x-40, c.y), false)
            disable_handle()
            _handle.update_canvas(0)
        }
        update_scale()
    }

    function enable_handle(){
        _handle.opacity = 1
    }

    function disable_handle(){
        _handle.cumulativeAngle=0
        _handle.update_canvas(0)
        _handle.opacity = 0
    }

    function reposition_add_markers(){
        var _path = path.slice(0,path.length-1)
        for (var i = 0; i< _path.length; i++)
            add_markers[i].center = Helper.geo_midpoint(_path[i],
                                                _path[Helper.add_and_wrap(i,_path.length)])
    }

    function reposition_vertex_markers(){
        var _path = path.slice(0,path.length-1)
        for (var i = 0; i< _path.length; i++)
            vertex_markers[i].center = _path[i]
    }


    function reposition_markers(){
        reposition_add_markers()
        reposition_vertex_markers()
    }

    function enable_markers(){
        for (var i = 0; i< vertex_markers.length; i++)
            vertex_markers[i].opacity = 1
        for (var i = 0; i< add_markers.length; i++)
            add_markers[i].opacity = 1
    }

    function enable_add_markers(){
        for (var i = 0; i< add_markers.length; i++)
            add_markers[i].opacity = 1
    }

    function disable_markers(){
        for (var i = 0; i< vertex_markers.length; i++)
            vertex_markers[i].opacity = 0
        for (var i = 0; i< add_markers.length; i++)
            add_markers[i].opacity = 0
    }

    function disable_add_markers(){
        for (var i = 0; i< add_markers.length; i++)
            add_markers[i].opacity = 0
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

    function map_polygon_point_mod_admissibility(pc, idx){
        var _path = path.slice(0,path.length-1)
        var pa = map.fromCoordinate(_path[Helper.add_and_wrap(idx,_path.length,2)])
        var pb = map.fromCoordinate(_path[Helper.add_and_wrap(idx,_path.length,1)])
        var pd = map.fromCoordinate(_path[Helper.sub_and_wrap(idx,_path.length,1)])
        var pe = map.fromCoordinate(_path[Helper.sub_and_wrap(idx,_path.length,2)])
        return ((Helper.three_point_direction(pa,pb,pc) === Helper.three_point_direction(pb,pc,pd)) &&
                Helper.three_point_direction(pb,pc,pd) === Helper.three_point_direction(pc,pd,pe))
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

    property var moving_idx: -1
    property var marked: -1
    property bool translating: false
    property bool rotating: false
    onTranslatingChanged: function(){
        reposition_markers()
        return (translating||rotating) ? disable_markers() : enable_markers()
    }
    onRotatingChanged: function(){
        reposition_markers();
        return (translating||rotating) ? disable_markers() : enable_markers()
    }
    function pos_changed_mod_handler(mouse){
        var p = Qt.point(mouse.x, mouse.y)
        var pf = map.toCoordinate(p)
        var _path = path.slice(0,path.length-1)
        if (moving_idx === -1 && !translating && !rotating){
            var thresh = 7
            var nearest = -1
            for (var i=0; i<_path.length; i++){
                var v = map.fromCoordinate(vertex_markers[i].center)
                var d = Helper.distance(p,v)
                if(d < thresh){
                    thresh = d
                    nearest = i
                }
                v = map.fromCoordinate(add_markers[i].center)
                d = Helper.distance(p,v)
                if(d < thresh){
                    thresh = d
                    nearest = i+_path.length
                }
            }
            v = map.fromCoordinate(_handle.h_center)
            d = Helper.distance(p,v)
            if(d < thresh){
                thresh = d
                nearest = 2*_path.length
            }
            v = map.fromCoordinate(_handle.h_handle)
            d = Helper.distance(p,v)
            if(d < thresh){
                thresh = d
                nearest = 2*_path.length+1
            }
            if (marked>=0 && marked < _path.length){
                vertex_markers[marked].color = "#00ff00"
                _dashed_line.replaceCoordinate(0, QtPositioning.coordinate(0,0))
                _dashed_line.replaceCoordinate(1, QtPositioning.coordinate(0,0))
            }
            else if(marked >= _path.length && marked < 2*_path.length){
                add_markers[marked-_path.length].color = "#bbbb00"
            }
            marked = nearest
            if (marked>=0 && marked < _path.length){
                vertex_markers[marked].color = "#ff0000"
                if (_path.length > 3){
                    _dashed_line.replaceCoordinate(0, _path[Helper.add_and_wrap(marked, _path.length)])
                    _dashed_line.replaceCoordinate(1, _path[Helper.sub_and_wrap(marked, _path.length)])
                }
            } else if(marked >= _path.length && marked < 2*_path.length){
                add_markers[marked-_path.length].color = "#ff0000"
            }
            _handle.center_select(marked === 2*_path.length)
            _handle.handle_select(marked === 2*_path.length+1)
        } else if (moving_idx>=0 && moving_idx<_path.length){
            var color = "#81c784"
            if (!map_polygon_point_mod_admissibility(p, moving_idx)){
                color = "#ffb300"
                pf = coordinateAt(moving_idx)
            }

            line.color = color
            replaceCoordinate(moving_idx, pf)
            vertex_markers[moving_idx].center = pf
            if (moving_idx === 0)
                replaceCoordinate(pathLength()-1, pf)
            update_centroid()
        } else if (translating){
            for (var i = 0; i<path.length; i++){
                var c = coordinateAt(i)
                var cn = QtPositioning.coordinate(c.latitude-centroid.latitude+pf.latitude,
                                                  c.longitude-centroid.longitude+pf.longitude)
                replaceCoordinate(i,cn)
            }
            update_centroid()
        } else if (rotating){
            var scale = _handle.h_center.distanceTo(pf)/_handle.h_center.distanceTo(_handle.h_handle)
            var angle = _handle.h_center.azimuthTo(pf)-_handle.h_center.azimuthTo(_handle.h_handle)
            for (var i = 0; i<path.length; i++){
                var c = coordinateAt(i)
                var d = centroid.distanceTo(c)
                var a = centroid.azimuthTo(c)
                var cn = centroid.atDistanceAndAzimuth(d*scale, a+angle)
                replaceCoordinate(i, cn)
            }
            _handle.update_canvas(angle)
            console.log(angle + "|" + Helper.deg_to_rad(angle))
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
        _dashed_line.replaceCoordinate(0, QtPositioning.coordinate(0,0))
        _dashed_line.replaceCoordinate(1, QtPositioning.coordinate(0,0))
        var p = Qt.point(mouse.x, mouse.y)
        var pf = map.toCoordinate(p)
        if (moving_idx === -1){
            var thresh = 7
            var nearest = -1
            var _path = path.slice(0,path.length-1)
            for (var i=0; i<_path.length; i++){
                var v = map.fromCoordinate(_path[i])
                var d = Helper.distance(p,v)
                if(d < thresh){
                    thresh = d
                    nearest = i
                }
                v = map.fromCoordinate(add_markers[i].center)
                d = Helper.distance(p,v)
                if(d < thresh){
                    thresh = d
                    nearest = i+_path.length
                }
            }
            v = map.fromCoordinate(_handle.h_center)
            d = Helper.distance(p,v)
            if(d < thresh){
                thresh = d
                nearest = -1
                translating = !translating
            }
            v = map.fromCoordinate(_handle.h_handle)
            d = Helper.distance(p,v)
            if(d < thresh){
                thresh = d
                nearest = -1
                translating = false
                rotating = !rotating
            }
            if (mouse.button & Qt.LeftButton){
                moving_idx = nearest
                if(moving_idx >=0 && moving_idx < _path.length){
                    disable_add_markers()
                    disable_handle()
                } else if (moving_idx >= _path.length){
                    var _idx = moving_idx-_path.length
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
                    enable_markers()
                    disable_add_markers()
                    disable_handle()
                    moving_idx = idx
                }
            } else if (mouse.button & Qt.RightButton){
                if (_path.length > 3 && nearest >=0 && nearest < _path.length){
                    removeCoordinate(nearest)
                    if (nearest == 0)
                        replaceCoordinate(pathLength()-1, coordinateAt(0))
                    map.removeMapItem(add_markers[nearest])
                    map.removeMapItem(vertex_markers[nearest])
                    add_markers.splice(nearest, 1)
                    vertex_markers.splice(nearest, 1)
                    reposition_add_markers()
                    update_centroid()
                }
            }
        } else {
            moving_idx = -1
            reposition_add_markers()
            enable_add_markers()
            enable_handle()
        }
        mapMouseArea.hoverEnabled = true
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

        intersections_cartesian = Helper.convex_polygon_parallel_slices_intersections(angle, offset, sides, lam/lom)

        if (method === "simple" || method === "single_winding")
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

    property var backup_path
    property var backup_vertex_markers
    property var backup_add_markers
    function begin_edit(){
        _canvas.canvasCtx.clearRect(0, 0, _canvas.canvasWidth, _canvas.canvasHeight)
        _canvas.requestPaint()
        enable_markers()
        enable_handle()
        backup_path=path
        backup_vertex_markers = vertex_markers
        backup_add_markers = add_markers

    }

    function discard_edit(){
        moving_idx = -1
        path=backup_path
        vertex_markers = backup_vertex_markers
        add_markers = backup_add_markers
        disable_markers()
        disable_handle()
        generate_path()
        draw_path()
        generate_nurbs()
    }

    function confirm_edit(angle, offset){
        angle=angle
        offset=offset
        disable_markers()
        disable_handle()
        generate_path()
        draw_path()
        generate_nurbs()
    }

    function draw_path(){
        var lam = Helper.lat_to_m_coeff(centroid.latitude)
        var lom = Helper.lon_to_m_coeff(centroid.longitude)

        // clear the canvas
        _canvas.canvasCtx.clearRect(0, 0, _canvas.canvasWidth, _canvas.canvasHeight)

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
        console.log(JSON.stringify(result))
        return result
    }
}
