import QtQuick 2.0
import QtLocation 5.6

MapPolyline {
    line.width: 3
    line.color: "#81c784"
    opacity: 1.0
    z: map.z + 5

    signal end

    property var polygonal_phase: 0
    property var poligonal_direction: 0 //1: counterclockwise, 2: clockwise

    function map_polygon_point_admissibility(pc){
        if (polygonal_phase === 3){
            var pa = map.fromCoordinate(coordinateAt(pathLength()-3))
            var pb = map.fromCoordinate(coordinateAt(pathLength()-2))
            var pd = map.fromCoordinate(coordinateAt(0))
            var pe = map.fromCoordinate(coordinateAt(1))
            return (poligonal_direction === three_point_direction(pa,pb,pc)
            &&  poligonal_direction === three_point_direction(pb,pc,pd)
            &&  poligonal_direction === three_point_direction(pc,pd,pe))
        } else return true
    }

    function three_point_direction(p1,p2,p3){
        var a1 = Math.atan2(p1.y-p2.y, p1.x-p2.x)*360/(2*Math.PI)
        var a2 = Math.atan2(p2.y-p3.y, p2.x-p3.x)*360/(2*Math.PI)
        a1 = a1-360*Math.floor(a1/360)
        a2 = a2-360*Math.floor(a2/360)
        return ((a1<a2 && (a2-a1)<180) || (a1>a2 && (a1-a2)>180)) ? 1/*cc*/ : 2/*c*/
    }

    function close_polygon(){
        if (polygonal_phase === 3){
            replaceCoordinate(pathLength()-1, coordinateAt(0))
            line.color = "#33cc33"
            polygonal_phase = 0
            mapMouseArea.hoverEnabled = false
            end()
        }
    }

    function pos_changed_handler(mouse){
        if (polygonal_phase === 0) return;
        var p = Qt.point(mouse.x, mouse.y)
        var pf = map.toCoordinate(Qt.point(mouse.x, mouse.y))
        var last_idx = pathLength()-1
        var color = "#33cc33"
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
                poligonal_direction = three_point_direction(p1,p2,p3)
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
}
