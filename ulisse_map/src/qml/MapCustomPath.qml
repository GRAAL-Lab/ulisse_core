import QtQuick 2.0
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls 2.1
import "."
import "../scripts/helper.js" as Helper

MapPolyline {
    line.width: 2
    line.color: lightgreen
    opacity: 1.0
    z: map.z + 5

    property bool multichoice: false

    property string type: "PointPath"
    property string pathName: "Path"

    signal end

    property Component mapMarkerComponent
    property Component mapMarkerLetterComponent
    property Component mapDashedLineComponent
    property Component mapHandleComponent
    property MapDashedLine _dashed_line
    property MapMarker _marker
    property MapHandle _handle
    property var vertex_markers: []
    property var add_markers: []
    property var a_marker
    property var b_marker

    property var direction: 0

    function toggle_dir() {
        direction = (++direction) % 2
        var t = QtPositioning.coordinate(b_marker.coordinate.latitude,
                                         b_marker.coordinate.longitude)
        b_marker.coordinate = QtPositioning.coordinate(
                    a_marker.coordinate.latitude, a_marker.coordinate.longitude)
        a_marker.coordinate = t
    }

    Component.onCompleted: {
        Helper.init_lib(QtPositioning)

        mapMarkerComponent = Qt.createComponent("MapMarker.qml")
        mapMarkerLetterComponent = Qt.createComponent("MapMarkerLetter.qml")
        mapDashedLineComponent = Qt.createComponent("MapDashedLine.qml")
        mapHandleComponent = Qt.createComponent("MapHandle.qml")

        _marker = mapMarkerComponent.createObject(map)
        _dashed_line = mapDashedLineComponent.createObject(map)
        _handle = mapHandleComponent.createObject(map)
        a_marker = mapMarkerLetterComponent.createObject(map)
        b_marker = mapMarkerLetterComponent.createObject(map)
        a_marker.opacity = 0
        b_marker.opacity = 0
        a_marker.content = "A";
        b_marker.content = "B";

        map.addMapItem(_marker)
        map.addMapItem(_dashed_line)
        map.addMapItem(_handle)
        map.addMapItem(a_marker)
        map.addMapItem(b_marker)
        _handle.add_to_map(map)
    }

    function deregister_map_items() {
        map.removeMapItem(_marker)
        map.removeMapItem(_dashed_line)
        map.removeMapItem(_handle)
        map.removeMapItem(a_marker)
        map.removeMapItem(b_marker)
        for (var i = 0; i < vertex_markers.length - 1; i++)
            map.removeMapItem(vertex_markers[i])
        for (var i = 0; i < add_markers.length - 1; i++)
            map.removeMapItem(add_markers[i])

        _handle.deregister_map_items(map)
    }

    function _generate_and_draw() {
        generate_nurbs()
        generate_markers()
        reposition_markers()
        disable_markers()
        disable_handle()
    }

    function generate_and_draw_deferred() {
        _generate_and_draw()
    }

    function get_path() {
        return path
    }

    function get_next_in_path(pos, plus) {
        var _path = get_path()
        return _path[Helper.add_and_wrap(pos, _path.length, plus)]
    }

    function get_prev_in_path(pos, minus) {
        var _path = get_path()
        return _path[Helper.sub_and_wrap(pos, _path.length, minus)]
    }

    function generate_markers() {
        var _path = get_path()
        while (add_markers.length > 0)
            map.removeMapItem(add_markers.pop())
        while (vertex_markers.length > 0)
            map.removeMapItem(vertex_markers.pop())
        add_markers = []
        vertex_markers = []
        for (var i = 0; i < _path.length; i++) {
            var marker1 = mapMarkerComponent.createObject(map)
            map.addMapItem(marker1)
            vertex_markers.push(marker1)
        }
        for (var i = 0; i < _path.length - 1; i++) {
            var marker2 = mapMarkerComponent.createObject(map)
            map.addMapItem(marker2)
            add_markers.push(marker2)
        }
        disable_handle()
        update_scale()
    }

    function reposition_handle() {
        var c = map.fromCoordinate(centroid, false)
        _handle.h_center = centroid
        _handle.h_handle = map.toCoordinate(Qt.point(c.x - 40, c.y), false)
        _handle.cumulativeAngle = 0
        _handle.add_angle(0)
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
                                                        get_next_in_path(i, 1))
            add_markers[i].opacity = 0
            add_markers[i].color = orange
        }
    }

    function reposition_markers() {
        reposition_add_markers()
        reposition_vertex_markers()
        a_marker.coordinate = path[0]
        b_marker.coordinate = path[path.length - 1]
        if (direction === 1) toggle_dir()
//        direction = 1
    }

    function enable_handle() {
        _handle.opacity = 1
    }

    function disable_handle() {
        _handle.cumulativeAngle = 0
        _handle.add_angle(0)
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
        for (var i = 0; i < vertex_markers.length; i++)
            vertex_markers[i].opacity = 1
    }

    function enable_add_markers() {
        for (var i = 0; i < add_markers.length; i++)
            add_markers[i].opacity = 1
    }

    function disable_markers() {
        disable_add_markers()
        disable_vertex_markers()
    }

    function disable_vertex_markers() {
        for (var i = 0; i < vertex_markers.length; i++)
            vertex_markers[i].opacity = 0
    }

    function disable_add_markers() {
        for (var i = 0; i < add_markers.length; i++)
            add_markers[i].opacity = 0
    }

    function clear_path() {
        for (var i = 0; i < path.length; i++)
            removeCoordinate(0)
    }

    function check_safe(box) {
        var bpp = [], ppp = []
        for (var i = 0; i < box.path.length; i++)
            bpp.push(map.fromCoordinate(box.path[i], false))
        for (var i = 0; i < path.length; i++)
            ppp.push(map.fromCoordinate(path[i], false))
        safe = Helper.polylines_disjoint(bpp, ppp)
                && Helper.coord_inside_polygon(path[0], box.path)
        return safe
    }

    property var safe: null
    onSafeChanged: function () {
        line.color = (safe ? lightgreen : red)
    }

    property var centroid

    property var backup_path
    property var backup_centroid
    property var backup_vertex_markers
    property var backup_add_markers

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
        } else if (marked >= _path.length && marked < 2 * _path.length - 1) {
            add_markers[marked - _path.length].color = orange
        }
        marked = nearest
        if (marked >= 0 && marked < _path.length) {
            vertex_markers[marked].color = red
            if (marked > 1 && marked < _path.length - 1) {
                _dashed_line.replaceCoordinate(0, get_next_in_path(marked, 1))
                _dashed_line.replaceCoordinate(1, get_prev_in_path(marked, 1))
            }
        } else if (marked >= _path.length && marked < 2 * _path.length - 1) {
            add_markers[marked - _path.length].color = red
        }
        _handle.center_select(marked === 2 * _path.length)
        _handle.handle_select(marked === 2 * _path.length + 1)
    }

    function change_coordinate(idx, coord) {
        replaceCoordinate(idx, coord)
        vertex_markers[idx].center = coord
        update_centroid()
    }

    function remove_coordinate(idx) {
        removeCoordinate(idx)
        if (idx < path.length - 1) {
            map.removeMapItem(add_markers[idx])
            add_markers.splice(idx, 1)
        }
        map.removeMapItem(vertex_markers[idx])
        vertex_markers.splice(idx, 1)
        reposition_add_markers()
        update_centroid()
    }

    function add_mid_coordinate(_idx) {
        var idx = _idx + 1
        add_markers[_idx].color = orange
        var c = add_markers[_idx].center
        insertCoordinate(idx, c)
        var marker1 = mapMarkerComponent.createObject(map)
        map.addMapItem(marker1)
        marker1.center = c
        marker1.opacity = 1
        marker1.color = green
        vertex_markers.splice(idx, 0, marker1)
        var marker2 = mapMarkerComponent.createObject(map)
        map.addMapItem(marker2)
        marker2.center = c
        marker2.opacity = 0
        marker2.color = orange
        add_markers.splice(idx, 0, marker2)
    }

    function move_coordinates(center) {
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
        var c_p = map.fromCoordinate(centroid)
        _handle.h_center = centroid
        _handle.h_handle = map.toCoordinate(Qt.point(c_p.x - 40, c_p.y), false)
    }

    function update_scale() {
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
                           && nearest < 2 * _path.length - 1) {
                    add_mid_coordinate(nearest - _path.length)
                    enable_markers()
                    disable_add_markers()
                    disable_handle()
                    moving_idx = nearest - _path.length + 1
                } else if (nearest === 2 * _path.length) {
                    translating = !translating
                } else if (nearest === 2 * _path.length + 1) {
                    rotating = !rotating
                }
            } else if (mouse.button & Qt.RightButton) {
                if (nearest >= 1 && nearest < _path.length - 1) {
                    remove_coordinate(nearest)
                    enable_add_markers()
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
            change_coordinate(moving_idx, pf)
        } else if (translating) {
            move_coordinates(map.toCoordinate(p))
        } else if (rotating) {
            var scale = _handle.h_center.distanceTo(
                        pf) / _handle.h_center.distanceTo(_handle.h_handle)
            var angle = _handle.h_center.azimuthTo(
                        pf) - _handle.h_center.azimuthTo(_handle.h_handle)
            rotate_and_scale_coordinates(angle, scale)
            _handle.h_handle = pf
        }
    }

    function begin_edit() {
        mapMouseArea.hoverEnabled = true
        moving_idx = -1
        backup_path = path
        backup_centroid = centroid
        backup_vertex_markers = vertex_markers
        backup_add_markers = add_markers
        reposition_markers()
        reposition_handle()
        enable_markers()
        enable_handle()
        disable_ab_markers()
    }

    function discard_edit() {
        mapMouseArea.hoverEnabled = false
        moving_idx = -1
        translating = false
        rotating = false
        path = backup_path
        vertex_markers = backup_vertex_markers
        add_markers = backup_add_markers
        centroid = backup_centroid
        reposition_markers()
        disable_markers()
        disable_handle()
        generate_nurbs()
        enable_ab_markers()
    }

    function confirm_edit(name, params) {
        pathName = name
        mapMouseArea.hoverEnabled = false
        moving_idx = -1
        _generate_and_draw()
        enable_ab_markers()
    }

    function click_handler(mouse) {
        if (mouse.button & Qt.LeftButton) {
            var p = Qt.point(mouse.x, mouse.y)
            var wp = map.toCoordinate(p)
            if (pathLength() === 0) {
                mapMouseArea.hoverEnabled = true
                addCoordinate(wp)
                addCoordinate(wp)
                line.color = orange
                return
            } else if (pathLength() > 1) {
                var lastwp = coordinateAt(pathLength() - 2)
                var lastp = map.fromCoordinate(lastwp)
                if (Helper.distance(p, lastp) < 8) {
                    removeCoordinate(pathLength() - 1)
                    line.color = lightgreen
                    mapMouseArea.hoverEnabled = false
                    update_centroid()
                    generate_nurbs()
                    end()
                    return
                }
            }
            line.color = orange
            addCoordinate(wp)
        }
    }

    function pos_changed_handler(mouse) {
        var p = Qt.point(mouse.x, mouse.y)
        var wp = map.toCoordinate(p)
        replaceCoordinate(pathLength() - 1, wp)
        if (pathLength() > 1) {
            var lastwp = coordinateAt(pathLength() - 2)
            var lastp = map.fromCoordinate(lastwp)
            if (Helper.distance(p, lastp) < 8) {
                line.color = orange
            } else {
                line.color = lightgreen
            }
        } else {
            line.color = lightgreen
        }
    }

    function generate_nurbs() {
        var centroid = Helper.coords_centroid(path)
        var points = Helper.points_map2euclidean(path, centroid)

        var nurb_l = []

        for (var i = 0; i < points.length-1; i++)
                nurb_l.push(Helper.generate_nurb_line(points[i], points[i+1],centroid))

        var result = {
            centroid: [centroid.latitude, centroid.longitude],
            curves: nurb_l,
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

            },
            values: values
        }
    }

    function deserialize(data) {
        var lat, lon
        pathName = data.name
        for (var j = 0; j < data.values.length; j++) {
            lat = data.values[j].latitude
            lon = data.values[j].longitude
            addCoordinate(QtPositioning.coordinate(lat, lon))
        }
    }

    function get_params() {
        return {
            name: pathName,
            params: {

            }
        }
    }

    function highlighted(yes) {
        line.color = yes ? orange : green
    }
}
