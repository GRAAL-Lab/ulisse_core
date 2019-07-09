import QtQuick 2.0
import QtLocation 5.6

MapPolyline {
    line.width: 3
    line.color: "#81c784"
    opacity: 1.0
    z: map.z + 5

    signal end

    property var rect_phase: 0

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

    function click_handler(mouse){
        if (mouse.button & Qt.LeftButton){
            if (rect_phase === 0){
                for(var i=0; i<5; i++)
                    removeCoordinate(0)
                var p = map.toCoordinate(Qt.point(mouse.x, mouse.y))
                addCoordinate(p)
                addCoordinate(p)
                mapMouseArea.hoverEnabled = true
                rect_phase = 1
            } else if (rect_phase === 1){
                var p = map.toCoordinate(Qt.point(mouse.x, mouse.y))
                addCoordinate(p)
                addCoordinate(coordinateAt(0))
                addCoordinate(coordinateAt(0))
                rect_phase = 2
            } else if (rect_phase === 2){
                rect_phase = 0
                mapMouseArea.hoverEnabled = false
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
}
