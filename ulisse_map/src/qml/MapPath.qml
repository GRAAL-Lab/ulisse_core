import QtQuick 2.0
import QtLocation 5.6

MapPolyline {
    line.width: 3
    line.color: "#81c784"
    opacity: 1.0
    z: map.z + 5

    signal end

    function distance(p1,p2){
        return Math.sqrt(Math.pow(p1.x-p2.x,2) + Math.pow(p1.y-p2.y,2))
    }

    function click_handler(mouse){
        if (mouse.button & Qt.LeftButton) {
            var p = Qt.point(mouse.x, mouse.y)
            var wp = map.toCoordinate(p)
            if (pathLength() === 0){
                mapMouseArea.hoverEnabled = true
                addCoordinate(wp)
                addCoordinate(wp)
                return
            }
            else if (pathLength() > 1){
                var lastwp = coordinateAt(pathLength()-2)
                var lastp = map.fromCoordinate(lastwp)
                if(distance(p, lastp) < 8){
                    removeCoordinate(pathLength()-1)
                    mapMouseArea.hoverEnabled = false
                    end()
                    return
                }
            }
            addCoordinate(wp)
        }
    }

    function pos_changed_handler(mouse){
        var p = Qt.point(mouse.x, mouse.y)
        var wp = map.toCoordinate(p)
        replaceCoordinate(pathLength()-1, wp)
    }
}
