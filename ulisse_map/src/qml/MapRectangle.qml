import QtQuick 2.0
import QtLocation 5.6

MapPolyline {
    line.width: 3
    line.color: "#81c784"
    opacity: 1.0
    z: map.z + 5

    signal end

    property var rect_phase: 0
    property var w
    property var h

    function project(p0,m,p1){
        var p2 = Qt.point(0,0);
        if ( (Math.abs(m) === 16331239353195370) || (Math.abs(m) === Infinity)){
            p2.x = p0.x
            p2.y = p1.y
        } else if (m === 0){
            p2.x = p1.x
            p2.y = p0.y
        } else {
            var m_perp = -1/m
            p2.x = (m*p0.x - m_perp*p1.x + p1.y - p0.y)/(m-m_perp)
            p2.y = m*(p2.x - p0.x) + p0.y
        }
        return p2
    }

    function distance(p1,p2){
        return Math.sqrt(Math.pow(p1.x-p2.x,2) + Math.pow(p1.y-p2.y,2))
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
                p1 = project(p0,m,p1)
            }
            replaceCoordinate(1, map.toCoordinate(p1))
        } else if (rect_phase === 2){
            var p0 = map.fromCoordinate(coordinateAt(0));
            var p1 = map.fromCoordinate(coordinateAt(1));
            var m = (p1.y - p0.y)/(p1.x - p0.x)
            var m_perp = -1/m
            var pm = Qt.point(mouse.x, mouse.y)
            var p2 = project(p1, m_perp, pm)
            var p3 = project(p0, m_perp, pm)
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

    property var points: [/*Qt.point(),Qt.point(),Qt.point(), Qt.point()*/]
    property var upper_side:  [/*Qt.point(), Qt.point()*/]
    property var left_side:   [/*Qt.point(), Qt.point()*/]
    property var right_side:  [/*Qt.point(), Qt.point()*/]
    property var lower_side: [/*Qt.point(), Qt.point()*/]
    property var sides: [upper_side, left_side, right_side, lower_side]
    property var angle: 30
    property var offset: 5
    property var coordinates: []


    function side_angle(side){
        var a = (360.0/(2*Math.PI))*Math.atan((side[0].y-side[1].y)/(side[0].x-side[1].x))
        return (a>0) ? a : a+90
    }

    function interpolate(from, to, offset, times){
        var a = Math.atan2(from.y - to.y, from.x - to.x)
        var ox = offset * Math.cos(a) * times
        var oy = offset * Math.sin(a) * times
        return Qt.point(from.x-ox, from.y-oy)
    }

    function intersections(point, angle, sides){
        var m0 = Math.tan((2*Math.PI/360.0)*angle)
        var q0 = point.y -m0*point.x
        var l0 = Qt.vector3d(m0, -1, q0)

        var ii = []
        for (var i=0; i<sides.length; i++){
            var s = sides[i]
            var mi = (s[0].y - s[1].y)/(s[0].x - s[1].x)
            var qi = s[0].y - mi*s[0].x
            var li = Qt.vector3d(mi, -1, qi)

            var intersection = l0.crossProduct(li)
            if (intersection.z === 0){/*parallel*/}
            else{
                var i_x = intersection.x/intersection.z
                var i_y = intersection.y/intersection.z
                if (((s[0].x <= i_x && i_x <= s[1].x) || s[1].x <= i_x && i_x <= s[0].x)
                &&  ((s[0].y <= i_y && i_y <= s[1].y) || s[1].y <= i_y && i_y <= s[0].y)){
                    ii.push(Qt.point(i_x, i_y))
                }
            }
        }
        return ii
    }

    function rectify_parallelogram_side(p0,p1,p2,p3){
        //p0-p1, p2-p3 must be parallel!
        //either p1 or p2 will be translated in order to create 90 degrees angles between p1 and p2, p2 and p3
        var _a = p1.x-p0.x
        var _b = p1.y-p0.y
        var _c = p1.x-p2.x
        var _d = p1.y-p2.y
        var a = Math.acos((_a*_c+_b*_d)/(Math.sqrt(_a*_a+_b*_b)*Math.sqrt(_c*_c+_d*_d)))
        var m = (p0.y - p1.y)/(p0.x - p1.x)
        if (a > Math.PI/2){
            p1 = project(p0, m, p2)
        } else {
            p2 = project(p3, m, p1)
        }
        return [p0, p1, p2, p3]
    }

    function map_to_canvas(){
        for (var i = 0; i<4; i++){
            points[i]=map.fromCoordinate(coordinateAt(i))
        }
        //FIXME: caso angolo 0?
        var r, l, t, b
        var pr, pl, pt, pb
        r = l = points[0].x
        t = b = points[0].y
        pr=pl=pt=pb=points[0]
        for (var i = 1; i<4; i++){
            if (points[i].x > r){ r = points[i].x; pr = points[i]}
            if (points[i].x < l){ l = points[i].x; pl = points[i]}
            if (points[i].y > t){ t = points[i].y; pt = points[i]}
            if (points[i].y < b){ b = points[i].y; pb = points[i]}
        }
        points = [pr, pl, pt, pb]
        upper_side = [pt, pr]
        right_side = [pr, pb]
        lower_side = [pb, pl]
        left_side =  [pl, pt]
    }

    function draw(){
        _canvas.canvasWidth=w
        _canvas.canvasHeight=h
        _canvas.zoomLevel = map.zoomLevel
        _canvas.coordinate = map.toCoordinate(Qt.point(min_x(), min_y()))
        _canvas.anchorPoint.x = 0
        _canvas.anchorPoint.y = 0
        var ctx = _canvas.canvasCtx
        ctx.fillStyle = Qt.rgba(0.5, 0, 0, 0.5);
        ctx.fillRect(0, 0, w, h);
        _canvas.requestPaint()

        var au = angle
        var at = side_angle(upper_side)
        var offset_on_t = offset/Math.cos((2*Math.PI/360.0)*(90-au))

        var times = 0
        var ii = []
        while (true){
            ++times
            var p = interpolate(points[2], points[1], offset_on_t, times)
            var _i = intersections(p, at-au, sides) //NB two intersections always
            if (_i.length < 2) break;
            ii.push([_i[0], _i[1]])
        }


        var direction = 0 //alternates at each step
        for (var i = 0; i < times-2; i++){
            direction = (direction === 0) ? 1 : 0
            if (direction === 1){
                var rr = rectify_parallelogram_side(ii[i][0], ii[i][1], ii[i+1][1], ii[i+1][0])
                ii[i][1] = rr[1]
                ii[i+1][1] = rr[2]
            } else {
                var rr = rectify_parallelogram_side(ii[i][1], ii[i][0], ii[i+1][0], ii[i+1][1])
                ii[i][0] = rr[1]
                ii[i+1][0] = rr[2]
            }
        }

        //direction = 0
        for (var i = 0; i < times-1; i++){
            //direction = (direction === 0) ? 1 : 0
            ctx.beginPath()
            ctx.moveTo(ii[i][0].x-min_x(), ii[i][0].y-min_y());
            ctx.lineTo(ii[i][1].x-min_x(), ii[i][1].y-min_y());
            ctx.closePath()
            ctx.stroke()
            _canvas.requestPaint()
        }
    }

    function canvas_to_map(){}
}
