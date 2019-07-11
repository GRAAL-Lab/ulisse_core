import QtQuick 2.0
import QtLocation 5.6
import "../scripts/helper.js" as Helper

MapPolyline {
    line.width: 3
    line.color: "#81c784"
    opacity: 1.0
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
                map_to_canvas()
                draw()
                canvas_to_map()
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

    property var points: []
    property var upper_side:  []

    property var sides: []
    property var angle: 30
    property var offset: 5
    property var coordinates: []


    function side_angle(side){
        var a = Helper.rad_to_deg(Math.atan(Helper.slope(side[0], side[1])))
        return (a>0) ? a : a+90
    }

    function intersections(point, angle, sides){
        var m0 = Math.tan(Helper.deg_to_rad(angle))
        var q0 = point.y -m0*point.x
        var l0 = Helper.to_homogeneous_line(m0, q0)

        var ii = []
        for (var i=0; i<sides.length; i++){
            var s = sides[i]
            var mi = Helper.slope(s[0], s[1])
            var qi = s[0].y - mi*s[0].x
            var li = Helper.to_homogeneous_line(mi, qi)

            var intersection = l0.crossProduct(li)
            if (intersection.z !== 0){/*not parallel*/
                var pi = Helper.from_homogeneous_point(intersection)
                if (Helper.point_in_box(pi, s[0], s[1]))
                    ii.push(pi)
            }
        }
        return ii
    }

    property var pt
    property var pl

    function map_to_canvas(){
        for (var i = 0; i<4; i++){
            points[i]=map.fromCoordinate(coordinateAt(i))
        }
        //FIXME: caso angolo 0?
        var t = Helper.find_top(points)
        var pti = t[1]
        pt = t[0]
        pl = points[pti > 1 ? pti : points.length-1]

        for (var i=0, idx_a=pti-1, idx_b; i<points.length; i++){
            idx_a = Helper.add_and_wrap(idx_a, points.length)
            idx_b = Helper.add_and_wrap(idx_a, points.length)
            sides.push([points[idx_a], points[idx_b]])
        }
        upper_side = sides[0]
    }

    function init_canvas(){
        _canvas.canvasWidth=w
        _canvas.canvasHeight=h
        _canvas.zoomLevel = map.zoomLevel
        _canvas.coordinate = map.toCoordinate(Qt.point(min_x(), min_y()))
        _canvas.anchorPoint.x = 0
        _canvas.anchorPoint.y = 0
    }

    function draw_line(ctx,x1,y1,x2,y2){
        ctx.beginPath()
        ctx.moveTo(x1,y1);
        ctx.lineTo(x2,y2);
        ctx.closePath()
        ctx.stroke()
    }

    function draw(){
        init_canvas()
        var ctx = _canvas.canvasCtx
        ctx.fillStyle = Qt.rgba(0, 0.4, 0, 0.2)
        ctx.fillRect(0, 0, w, h)
        _canvas.requestPaint()

        var au_deg = angle
        var at_deg = side_angle(upper_side)
        var offset_on_t = offset/Math.cos(Helper.deg_to_rad(90-au_deg))

        var times = 0
        var ii = []
        while (true){
            ++times
            var p = Helper.interpolate(pt, pl, offset_on_t, times)
            var _i = intersections(p, at_deg-au_deg, sides) //NB two intersections always if a comvex polygon
            if (_i.length < 2) break
            ii.push([_i[0], _i[1]])
        }

        var a = 0, b = 1
        for (var i = 0; i < times-2; i++){
            var rr = Helper.rectify_parallelogram_side(ii[i][a], ii[i][b], ii[i+1][b], ii[i+1][a])
            ii[i][b] = rr[1]
            ii[i+1][b] = rr[2]
            a = Helper.flip(a)
            b = Helper.flip(b)
        }

        for (var i = 0; i < times-1; i++){
            var p0 = Helper.toCanvasCoordinates(ii[i][0], map, _canvas)
            var p1 = Helper.toCanvasCoordinates(ii[i][1], map, _canvas)
            draw_line(ctx, p0.x, p0.y, p1.x, p1.y)
            _canvas.requestPaint()
        }
    }

    function canvas_to_map(){}
}
