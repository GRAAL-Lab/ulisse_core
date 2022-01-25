import QtQuick 2.0
import QtLocation 5.9
import QtPositioning 5.9
import QtQuick.Controls.Material 2.1
import "."
import "../scripts/helper.js" as Helper


MapPolyline {
    id: root
    line.width: 2
    opacity: 1.0
    z: map.z + 5

    property string type: "PolyPath"

    signal end

    property var clickHandler: click_handler_convex
    property var posChangedHandler: pos_changed_handler_convex

    property Component mapCanvasComponent
    property Component mapMarkerComponent
    property Component mapMarkerLetterComponent
    property Component mapDashedLineComponent
    property Component mapHandleComponent

    property MapDashedLine _dashed_line
    property MapCanvas _canvas
    property MapMarker _marker
    property MapHandle _handle

    property var vertex_markers: []
    property var add_markers: []
    property var a_marker
    property var b_marker

    property string pathName: "Path"
    property real _angle: 30
    property real _offset: 30
    property var _method: "single_winding" // "simple"
    property var direction: 0

    property var centroid
    property var intersections_canvas: []
    property var intersections_cartesian: []
    property var intersections_geographic: []

    property var polygonal_phase: 0
    property var polygonal_direction: 0 //1: counterclockwise, 2: clockwise

    property var safe: null
    property var detection_intersect: 0

    onSafeChanged: function () {
        line.color = safe ? green : red
    }

    ////////////////////////////////////
    // Initializazion / deinitialization
    Component.onCompleted: {
        Helper.init_lib(QtPositioning)
        mapCanvasComponent = Qt.createComponent("MapCanvas.qml")
        mapMarkerComponent = Qt.createComponent("MapMarker.qml")
        mapDashedLineComponent = Qt.createComponent("MapDashedLine.qml")
        mapHandleComponent = Qt.createComponent("MapHandle.qml")
        mapMarkerLetterComponent = Qt.createComponent("MapMarkerLetter.qml")

        _canvas = mapCanvasComponent.createObject(map)
        _marker = mapMarkerComponent.createObject(map)
        _dashed_line = mapDashedLineComponent.createObject(map)
        _handle = mapHandleComponent.createObject(map)
        a_marker = mapMarkerLetterComponent.createObject(map)
        b_marker = mapMarkerLetterComponent.createObject(map)

        a_marker.content = "A";
        b_marker.content = "B";

        map.addMapItem(_canvas)
        map.addMapItem(_marker)
        map.addMapItem(_dashed_line)
        map.addMapItem(_handle)
        map.addMapItem(a_marker)
        map.addMapItem(b_marker)
        _handle.add_to_map(map)

    }

    onPathNameChanged: {
        console.log("[MapPolygon] PathName: " + pathName)
        console.log("[MapPolygon] PathType: " + type)
    }


    function deregister_map_items() {
        map.removeMapItem(_canvas)
        map.removeMapItem(a_marker)
        map.removeMapItem(b_marker)
        map.removeMapItem(_marker)
        map.removeMapItem(_dashed_line)
        map.removeMapItem(_handle)
        clear_markers()
        _handle.deregister_map_items(map)
    }

    //////////////////////////////////////////
    // Path query/editing as a circular buffer
    function get_path() {
        return path.slice(0, path.length - 1)
    }

    function next_in_path(pos, plus) {
        var _path = get_path()
        return _path[Helper.add_and_wrap(pos, _path.length, plus)]
    }

    function prev_in_path(pos, minus) {
        var _path = get_path()
        return _path[Helper.sub_and_wrap(pos, _path.length, minus)]
    }

    function clear_path() {
        while (path.length > 0)
            removeCoordinate(0)
        centroid = null
        detection_intersect = 0
    }

    function change_coordinate(idx, coord) {
        replaceCoordinate(idx, coord)
        if (idx === 0)
            replaceCoordinate(path.length - 1, coord)
        update_centroid()
    }

    function remove_coordinate(idx) {
        removeCoordinate(idx)
        if (idx === 0)
            replaceCoordinate(pathLength() - 1, path[0])
        update_centroid()
    }

    function split_edge(idx) {
        var c = Helper.geo_midpoint(get_path()[idx], next_in_path(idx, 1))
        insertCoordinate(idx+1, c)
    }

    function translate_coordinates(center) {
        for (var i = 0; i < path.length; i++) {
            var c = path[i]
            var cn = QtPositioning.coordinate(
                        c.latitude - centroid.latitude + center.latitude,
                        c.longitude - centroid.longitude + center.longitude)
            replaceCoordinate(i, cn)
        }
        update_centroid()
    }

    function rotate_and_scale_coordinates(angle, scale) {
        for (var i = 0; i < path.length; i++) {
            var c = path[i]
            var d = centroid.distanceTo(c)
            var a = centroid.azimuthTo(c)
            var cn = centroid.atDistanceAndAzimuth(d * scale, a + angle)
            replaceCoordinate(i, cn)
        }
        _handle.add_angle(angle)
    }

    function update_centroid() {
        var _path = get_path()
        centroid = Helper.coords_centroid(_path)
        reposition_handle()
    }

    function close_polygon() {
        replaceCoordinate(pathLength() - 1, path[0])
        mapMouseArea.hoverEnabled = false
        line.color = green
        polygonal_phase = 0
        end()
    }


    //////////////////////////////////
    // Markers management
    function generate_markers() {
        var _path = get_path()
        clear_markers()
        for (var i in _path) {
            var marker1 = mapMarkerComponent.createObject(map)
            var marker2 = mapMarkerComponent.createObject(map)
            map.addMapItem(marker1)
            map.addMapItem(marker2)
            vertex_markers.push(marker1)
            add_markers.push(marker2)
        }
        disable_handle()
        update_markers_scale()
    }

    function clear_markers(){
        while (add_markers.length > 0)
            map.removeMapItem(add_markers.pop())
        while (vertex_markers.length > 0)
            map.removeMapItem(vertex_markers.pop())
        add_markers = []
        vertex_markers = []
    }

    function reposition_handle() {
        var c = map.fromCoordinate(centroid, false)
        _handle.h_center = centroid
        _handle.h_handle = map.toCoordinate(Qt.point(c.x - 40, c.y), false)
        _handle.reset_angle()
    }

    function reposition_vertex_markers() {
        var _path = get_path()
        for (var i = 0; i < vertex_markers.length; i++) {
            vertex_markers[i].center = _path[i]
            vertex_markers[i].opacity = 0
            vertex_markers[i].color = green
        }
    }

    function reposition_add_markers() {
        var _path = get_path()
        for (var i = 0; i < add_markers.length; i++) {
            add_markers[i].center = Helper.geo_midpoint(_path[i],
                                                        next_in_path(i, 1))
            add_markers[i].opacity = 0
            add_markers[i].color = orange
        }
    }

    function reposition_markers() {
        reposition_add_markers()
        reposition_vertex_markers()
        a_marker.coordinate = intersections_geographic[0][0]
        b_marker.coordinate = intersections_geographic[intersections_geographic.length
                                                       - 1][(intersections_geographic.length
                                                             % 2 === 0) ? 0 : 1]
        if (direction === 1) toggle_dir()
    }

    function enable_handle() {
        _handle.opacity = 1
    }

    function disable_handle() {
        _handle.reset_angle()
        _handle.opacity = 0
    }

    function enable_markers() {
        enable_vertex_markers()
        enable_add_markers()
    }

    function disable_ab_markers() {
        a_marker.opacity = 0
        b_marker.opacity = 0
    }

    function enable_ab_markers() {
        a_marker.opacity = 0.75
        b_marker.opacity = 0.75
    }

    function enable_vertex_markers() {
        Helper.set_bulk_opacity(vertex_markers, 1)
    }

    function enable_add_markers() {
        Helper.set_bulk_opacity(add_markers, 1)
    }

    function disable_markers() {
        disable_add_markers()
        disable_vertex_markers()
    }

    function disable_vertex_markers() {
        Helper.set_bulk_opacity(vertex_markers, 0)
    }

    function disable_add_markers() {
        Helper.set_bulk_opacity(add_markers, 0)
    }

    function toggle_dir() {
        direction = Helper.flip(direction)
        var temp = Helper.coordinate_deep_copy(b_marker.coordinate)
        b_marker.coordinate = Helper.coordinate_deep_copy(a_marker.coordinate)
        a_marker.coordinate = temp
    }

    function nearest_marker(point, markers, nearest, thresh) {
        for (var i = 0; i < markers.length; i++) {
            var v = map.fromCoordinate(markers[i].center)
            var d = Helper.distance(point, v)
            if (d < thresh) {
                thresh = d
                nearest = i
            }
        }
        return {
            nearest: nearest,
            distance: thresh
        }
    }

    function nearest_than(p, d1, n1, p2, n2) {
        var d2 = Helper.distance(p, p2)
        return (d1 < d2) ? {
                               nearest: n1,
                               distance: d1
                           } : {
            nearest: n2,
            distance: d2
        }
    }

    function change_marked(nearest) {
        var _path = get_path()
        if (marked >= 0 && marked < _path.length) {
            vertex_markers[marked].color = green
            _dashed_line.reset()
        } else if (marked >= _path.length && marked < 2 * _path.length) {
            add_markers[marked - _path.length].color = orange
        }
        marked = nearest
        if (marked >= 0 && marked < _path.length) {
            vertex_markers[marked].color = red
            if (_path.length > 3) {
                _dashed_line.replaceCoordinate(0, next_in_path(marked, 1))
                _dashed_line.replaceCoordinate(1, prev_in_path(marked, 1))
            }
        } else if (marked >= _path.length && marked < 2 * _path.length) {
            add_markers[marked - _path.length].color = red
        }
        _handle.center_select(marked === 2 * _path.length)
        _handle.handle_select(marked === 2 * _path.length + 1)
    }

    function update_markers_scale() {
        var a = map.toCoordinate(Qt.point(0, 0))
        var b = map.toCoordinate(Qt.point(0, 1))
        var p2m = a.distanceTo(b)
        var r = 10 * p2m
        for (var i = 0; i < vertex_markers.length; i++)
            vertex_markers[i].radius = r
        for (var i = 0; i < add_markers.length; i++)
            add_markers[i].radius = r
        _handle.h_radius = r
        a_marker.zoomLevel = map.zoomLevel / 2 + 9
        b_marker.zoomLevel = map.zoomLevel / 2 + 9
    }

    //////////////////////////////////////////////
    // Event handlers for definition of simple, non-intersecting polygon (safety polygon)
    function click_handler_non_intersecting(mouse) {
        var m = Qt.point(mouse.x, mouse.y)
        var p = map.toCoordinate(m)
        if (mouse.button & Qt.LeftButton) {
            if (path.length === 0) {
                mapMouseArea.hoverEnabled = true
                addCoordinate(p)
            }
            if (!detection_intersect)
                addCoordinate(p)
        }
        if (mouse.button & Qt.RightButton) {
            if (path.length >= 3 && !detection_intersect) {
                var ppp = []
                for (var i = 0; i < path.length - 1; i++)
                    ppp.push(map.fromCoordinate(path[i], false))
                var closing_side = [ppp[0], ppp[ppp.length - 1]]
                if (!Helper.polylines_disjoint(closing_side, ppp))
                    return
                //var p1 = map.fromCoordinate(coordinateAt(0)) // (serve a qualcosa?)
                var pp = path
                pp.push(pp[0])
                if (!Helper.coord_inside_polygon(fbkUpdater.ulisse_pos, pp))
                    return
                console.log("[MapPolygon.click_handler_non_intersecting] close_polygon()")

                close_polygon()
                update_centroid()
                generate_markers()
            }
        }
    }

    function pos_changed_handler_simple(mouse) {
        var p = Qt.point(mouse.x, mouse.y)
        var pf = map.toCoordinate(p)
        var last_idx = pathLength() - 1
        var color = cyan

        var ppp = []
        detection_intersect = false
        for (var i = 0; i < path.length - 1; i++)
            ppp.push(map.fromCoordinate(path[i], false))
        if (path.length > 3) {
            var new_side = [p, ppp[ppp.length - 1]]
            if (!Helper.polylines_disjoint(new_side, ppp)) {
                color = red
                pf = coordinateAt(path.length - 1)
                detection_intersect = true
            }
        }
        line.color = color
        replaceCoordinate(last_idx, pf)
    }

    //////////////////////////////////////////////
    // Logic for definition of rectangle
    function click_handler_rect(mouse) {
        if (mouse.button & Qt.LeftButton) {
            if (polygonal_phase === 0) {
                var p = Qt.point(mouse.x, mouse.y)
                var wp = map.toCoordinate(p)
                addCoordinate(wp)
                addCoordinate(wp)
                mapMouseArea.hoverEnabled = true
                polygonal_phase = 1
            } else if (polygonal_phase === 1) {
                var p = Qt.point(mouse.x, mouse.y)
                var wp = map.toCoordinate(p)
                var wp0 = coordinateAt(0)
                addCoordinate(wp)
                addCoordinate(wp0)
                addCoordinate(wp0)
                polygonal_phase = 2
                line.color = orange
            } else if (polygonal_phase === 2) {
                var p0 = map.fromCoordinate(coordinateAt(0))
                var p2 = map.fromCoordinate(coordinateAt(2))
                polygonal_phase = 0
                mapMouseArea.hoverEnabled = false
                line.color = green
                end()
            }
        }
    }

    function pos_changed_handler_rect(mouse) {

        if (polygonal_phase === 1) {
            var p0 = map.fromCoordinate(coordinateAt(0))
            var p1 = Qt.point(mouse.x, mouse.y)
            if (mouse.modifiers & Qt.ShiftModifier) {
                var n_intervals = 16
                var snap_interval = 2 * Math.PI / n_intervals
                var theta = Math.atan2(p1.y - p0.y, p1.x - p0.x)
                var snap_idx = Math.floor((theta + snap_interval / 2) / snap_interval)
                var snap_theta = snap_idx * snap_interval
                var m = Math.tan(snap_theta)
                var centroid = Helper.coords_centroid(coords)
                var lam = Helper.lat_to_m_coeff(centroid.latitude)
                var lom = Helper.lon_to_m_coeff(centroid.longitude)
                p1 = Helper.project(p0, m, p1, lam / lom)
            }
            replaceCoordinate(1, map.toCoordinate(p1))
        } else if (polygonal_phase === 2) {
            var cp0 = coordinateAt(0)
            var cp1 = coordinateAt(1)
            var cpc = map.toCoordinate(Qt.point(mouse.x, mouse.y), false)
            var coords = [cp0, cp1, cpc]
            var centroid = Helper.coords_centroid(coords)
            var lam = Helper.lat_to_m_coeff(centroid.latitude)
            var lom = Helper.lon_to_m_coeff(centroid.longitude)
            var points = Helper.points_map2euclidean(coords, centroid)
            var m = Helper.slope(points[0], points[1])

            var m_perp = -(lam / lom)/ m //perpendicularity constraint for aspect ratios different than 1:1

            var p2 = Helper.intersect_two_lines(points[1], m_perp, points[2], m)
            var p3 = Helper.intersect_two_lines(points[0], m_perp, points[2], m)
            var cp2 = cmdWrapper.localUTM2LatLong(p2, centroid)
            var cp3 = cmdWrapper.localUTM2LatLong(p3, centroid)

            if (cp2 !== null && cp2.isValid && cp3 !== null && cp3.isValid) {
                replaceCoordinate(2, cp2)
                replaceCoordinate(3, cp3)
            }
        }
    }

    //////////////////////////////////////////////
    // Logic for definition of simple, convex polygon

    //FIXME: check it in the euclidean plane
    // used during polygon definition, supposing to close the polygon with
    // point pc as the last added point
    function map_polygon_point_admissibility(pc) {
        var pa = map.fromCoordinate(path[pathLength() - 3])
        var pb = map.fromCoordinate(path[pathLength() - 2])
        var pd = map.fromCoordinate(path[0])
        var pe = map.fromCoordinate(path[1])
        return Helper.coherent_points_direction([pa, pb, pc, pd, pe], polygonal_direction)
    }

    function pos_changed_handler_convex(mouse, box) {
        if (polygonal_phase === 0)
            return
        var p = Qt.point(mouse.x, mouse.y)
        var color
        var pf = map.toCoordinate(p)
        if (polygonal_phase === 3 && !map_polygon_point_admissibility(p)) {
            pf = path[0]
            color = orange
        } else {
            color = green
        }
        line.color = color
        replaceCoordinate(path.length - 1, pf)
    }

    function click_handler_convex(mouse) {
        var m = Qt.point(mouse.x, mouse.y)
        var p = map.toCoordinate(m)
        if (mouse.button & Qt.LeftButton) {
            if (polygonal_phase === 0) {
                clear_path()
                addCoordinate(p)
                addCoordinate(p)
                polygonal_phase = 1
                mapMouseArea.hoverEnabled = true
            } else if (polygonal_phase === 1) {
                addCoordinate(p)
                polygonal_phase = 2
            } else if (polygonal_phase === 2) {
                var p1 = map.fromCoordinate(path[0])
                var p2 = map.fromCoordinate(path[1])
                polygonal_direction = Helper.three_point_direction(p1, p2, m)
                addCoordinate(p)
                polygonal_phase = 3
            } else if (polygonal_phase === 3) {
                if (map_polygon_point_admissibility(m)){
                    addCoordinate(p)
                } else {
                    console.log("[MapPolygon.click_handler_convex] close_polygon()")
                    close_polygon()
                }
            }
        } else if (mouse.button & Qt.RightButton) {
            close_polygon()
        }
    }



    //////////////////////////////////////////////
    // Event handlers for polygon editing
    property var moving_idx: -1
    property var marked: -1
    property bool translating: false
    property bool rotating: false

    onTranslatingChanged: function () {
        reposition_markers()
        reposition_handle()
        return (translating || rotating) ? disable_markers() : enable_markers()
    }

    onRotatingChanged: function () {
        reposition_markers()
        reposition_handle()
        return (translating || rotating) ? disable_markers() : enable_markers()
    }

    //FIXME: check it in the euclidean plane
    //used during polygon edit, assuming pc as the point that is being dragged
    function map_polygon_point_mod_admissibility(pc, idx) {
        var pa = map.fromCoordinate(prev_in_path(idx, 2))
        var pb = map.fromCoordinate(prev_in_path(idx, 1))
        var pd = map.fromCoordinate(next_in_path(idx, 1))
        var pe = map.fromCoordinate(next_in_path(idx, 2))
        return Helper.coherent_points_direction([pa, pb, pc, pd, pe])
    }

    function click_mod_handler(mouse) {
        mapMouseArea.hoverEnabled = false
        _dashed_line.reset()
        var p = Qt.point(mouse.x, mouse.y)
        var pf = map.toCoordinate(p)
        var _path = get_path()
        if (moving_idx === -1) {
            var hpc = map.fromCoordinate(_handle.h_center)
            var hph = map.fromCoordinate(_handle.h_handle)
            var r1 = nearest_marker(p, add_markers, -1, 7)
            var r2 = nearest_marker(
                        p, vertex_markers,
                        (r1.nearest === -1) ? -1 : r1.nearest + _path.length,
                        r1.distance)
            var r3 = nearest_than(p, r2.distance,
                                  (r2.nearest === -1) ? -1 : r2.nearest, hpc,
                                  2 * _path.length)
            var r4 = nearest_than(p, r3.distance,
                                  (r3.nearest === -1) ? -1 : r3.nearest, hph,
                                  2 * _path.length + 1)
            var nearest = r4.nearest
            if (mouse.button & Qt.LeftButton) {
                if (nearest >= 0 && nearest < _path.length) {
                    disable_add_markers()
                    disable_handle()
                    moving_idx = nearest
                } else if (nearest >= _path.length
                           && nearest < 2 * _path.length) {
                    split_edge(nearest - _path.length)
                    generate_markers()
                    reposition_markers()
                    enable_vertex_markers()
                    disable_handle()
                    moving_idx = nearest - _path.length + 1
                } else if (nearest === 2 * _path.length) {
                    translating = !translating
                } else if (nearest === 2 * _path.length + 1) {
                    rotating = !rotating
                }
            } else if (mouse.button & Qt.RightButton) {
                if (nearest >= 0 && nearest < _path.length) {
                    if (_path.length > 3) {
                        remove_coordinate(nearest)
                        generate_markers()
                        reposition_markers()
                        enable_markers()
                    }
                }
            }
        } else if (moving_idx >= 0) {
            moving_idx = -1
            reposition_add_markers()
            enable_add_markers()
            enable_handle()
        }
        mapMouseArea.hoverEnabled = true
    }

    function pos_changed_mod_handler(mouse) {
        var p = Qt.point(mouse.x, mouse.y)
        var pf = map.toCoordinate(p)
        var _path = get_path()
        if (moving_idx === -1 && !translating && !rotating) {
            var hpc = map.fromCoordinate(_handle.h_center)
            var hph = map.fromCoordinate(_handle.h_handle)
            var r1 = nearest_marker(p, add_markers, -1, 7)
            var r2 = nearest_marker(
                        p, vertex_markers,
                        (r1.nearest === -1) ? -1 : r1.nearest + _path.length,
                        r1.distance)
            var r3 = nearest_than(p, r2.distance,
                                  (r2.nearest === -1) ? -1 : r2.nearest, hpc,
                                  2 * _path.length)
            var r4 = nearest_than(p, r3.distance,
                                  (r3.nearest === -1) ? -1 : r3.nearest, hph,
                                  2 * _path.length + 1)
            change_marked(r4.nearest)
        } else if (moving_idx >= 0 && moving_idx < _path.length) {
            if (map_polygon_point_mod_admissibility(p, moving_idx)) {
                color = green
            } else {
                color = orange
                pf = path[moving_idx]
            }
            line.color = color
            change_coordinate(moving_idx, pf)
            vertex_markers[moving_idx].center = pf
        } else if (translating) {
            translate_coordinates(map.toCoordinate(p))
        } else if (rotating) {
            var scale = _handle.h_center.distanceTo(
                        pf) / _handle.h_center.distanceTo(_handle.h_handle)
            var angle = _handle.h_center.azimuthTo(
                        pf) - _handle.h_center.azimuthTo(_handle.h_handle)
            rotate_and_scale_coordinates(angle, scale)
            _handle.h_handle = pf
        }
    }


    function generate_path() {
        if (!(_method === "simple" || _method === "single_winding"))
            return

        var orig_tilt = map.tilt
        map.tilt = 0

        var shape_coords = get_path()

        centroid = Helper.coords_centroid(shape_coords)
        var limits = Helper.shape_geo_limits(shape_coords)

        var lam = Helper.lat_to_m_coeff(centroid.latitude)
        var lom = Helper.lon_to_m_coeff(centroid.longitude)

        var dim = Helper.shape_px_dimensions(limits, map)

        // transform shape coordinates and work in a virtual euclidean metric system
        var points = Helper.points_map2euclidean(shape_coords, centroid)
        points = Helper.set_points_clockwise(points)
        var sides = Helper.make_sides(points)

        intersections_cartesian = Helper.convex_polygon_parallel_slices_intersections(_angle, _offset, sides, lam / lom)

        if (_method === "simple" || _method === "single_winding")
            intersections_cartesian = Helper.rectify_dense_winding(intersections_cartesian, lam / lom)

        // transform virtual euclidean reference points in map coordinates
        intersections_geographic = Helper.segments_euclidean2map(intersections_cartesian, centroid)

        var a = map.toCoordinate(Qt.point(0, 0))
        var b = map.toCoordinate(Qt.point(0, 1))
        var c = map.toCoordinate(Qt.point(1, 0))
        var p2m_h = a.distanceTo(b)
        var p2m_v = a.distanceTo(c)

        _offset *= 3
        var o_x = _offset / p2m_h
        var o_y = _offset / p2m_v

        Helper.init_canvas(
                    _canvas, map, dim[0] + 2 * o_x, dim[1] + 2 * o_y,
                    limits.max_lat + _offset / lam, limits.min_lon - _offset / lom,
                    Helper.relative_zoom_pixel_ratio(map, map.maximumZoomLevel))

        _offset /= 3

        // transform map coordinates in canvas pixel indexes
        intersections_canvas = Helper.segments_map2canvas(intersections_geographic, map, _canvas)
        map.tilt = orig_tilt
    }


    /////////////////////////////////////////////////
    // Editing/finalization utilities
    property var backup_path

    function begin_edit() {
        mapMouseArea.hoverEnabled = true
        moving_idx = -1
        backup_path = path
        reposition_markers()
        reposition_handle()
        enable_markers()
        enable_handle()
        disable_ab_markers()
        _canvas.clear_canvas()
    }

    function discard_edit() {
        mapMouseArea.hoverEnabled = false
        moving_idx = -1
        translating = false
        rotating = false
        path = backup_path
        update_centroid()
        generate_markers()
        reposition_markers()
        disable_markers()
        disable_handle()
        if (_method !== null || _method !== undefined) {
            // Insert C++ generation also here
            generate_path()
            generate_nurbs()
            draw_path()

        }
    }

    function confirm_edit(name, params) {
        console.log("[MapCustomPolygon] confirm edit")
        mapMouseArea.hoverEnabled = false
        pathName = name
        if (params !== null || params !== undefined) {
            _angle = params.angle
            _offset = params.offset
            _method = params.method
        }
        moving_idx = -1
        _generate_and_draw()
    }

    function get_params() {
        return {
            name: pathName,
            params: {
                angle: _angle,
                offset: _offset,
                method: _method
            }
        }
    }


    //////////////////////////////////////////////////////////
    // Draw utilities
    function _generate_and_draw() {
        if (_method != null || _method !== undefined) {
            // For the SafetyBoundary the _method is null

            console.log("[MapPolygon] generate_path()")
            //cmdWrapper.generatePath <----
            generate_path()
            console.log("[MapPolygon] draw_path()")
            draw_path()
        }

        generate_markers()
        reposition_markers()
        disable_markers()
        disable_handle()

        console.log("[MapPolygon] _generate_and_draw() DONE")
    }

    function generate_and_draw_deferred() {
        if (_canvas.canvasCtx !== null && _canvas.canvasCtx !== undefined){
            _generate_and_draw()
        }
        else {
            //console.log("[MapPolygon] generate_and_draw_deferred() - canvasCtx Undefined")
            _canvas.contextReady.connect(_generate_and_draw)
        }
    }

    function draw_path() {
        // clear the canvas
        _canvas.clear_canvas()
        Helper.draw_path_lines(_canvas, cmdWrapper.createNurbs(JSON.stringify(generate_nurbs())), map)

    }

    ////////////////////////////////////////////////////////
    function generate_nurbs() {
        // This function will be replaced since the sisl_toolbox will generate the curve
        var nurb_l = []
        var nurb_c = []
        for (var i = 0; i < intersections_cartesian.length; i++) {
            var p0 = intersections_cartesian[i][i % 2]
            var p1 = intersections_cartesian[i][(i + 1) % 2]
            nurb_l.push(Helper.generate_nurb_line(p0, p1, centroid))
        }

        for (var i = 0; i < intersections_cartesian.length - 1; i++) {
            var dir = (i + 1) % 2
            var p0 = intersections_cartesian[i][dir]
            var p3 = intersections_cartesian[i + 1][dir]
            if (_method === "simple")
                nurb_c.push(Helper.generate_nurb_line(p0, p3, centroid))
            else if (_method === "single_winding")
                nurb_c.push(Helper.generate_nurb_circle(p0, p3, dir, centroid))
        }


        var curves = [nurb_l[0]]
        for (var i = 0; i < intersections_cartesian.length - 1; i++) {
            curves.push(nurb_c[i])
            curves.push(nurb_l[i + 1])
        }

        var result = {
            centroid: [centroid.latitude, centroid.longitude],
            curves: curves,
            direction: direction
        }
        return result
    }

    function serialize() {
        var values = []
        for (var j = 0; j < path.length; j++) {
            var p_i = path[j]
            values.push({
                            latitude: p_i.latitude,
                            longitude: p_i.longitude
                        })
        }
        return {
            type: type,
            name: pathName,
            params: {
                offset: _offset,
                angle: _angle,
                method: _method
            },
            values: values
        }
    }

    function deserialize(data) {
        var lat, lon
        for (var j = 0; j < data.values.length; j++) {
            lat = data.values[j].latitude
            lon = data.values[j].longitude
            addCoordinate(QtPositioning.coordinate(lat, lon))
        }
        pathName = data.name
        _angle = data.params.angle
        _offset = data.params.offset
        _method = data.params.method
    }

    function get_nurbs(data){
        nurbs = data
    }

    ///////////////////////////////////////////////////
    // Safety
    function check_safe(box) {
        console.log("[MapCustomPolygon] check_safe()")
        var bpp = [], ppp = []
        for (var i in box.path) {
            bpp.push(map.fromCoordinate(box.path[i], false))

        }

        console.log("safety:" + bpp)

        for (var j in path) {
            ppp.push(map.fromCoordinate(path[j], false))
        }

        console.log("polypath:" + ppp)



        console.log("[MapCustomPolygon] check_safe()->pd")
        var pd = Helper.polylines_disjoint(bpp, ppp)
        console.log("[MapCustomPolygon] check_safe()->cip")
        var cip = Helper.coord_inside_polygon(path[0], box.path)

        safe = (pd && cip)

        return safe
    }


    function highlighted(yes) {
        var desel_line_color = safe ? lightgreen : red
        line.color = yes ? orange : desel_line_color
    }
}
