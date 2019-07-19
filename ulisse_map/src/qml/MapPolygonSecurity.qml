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

    property var w
    property var h

    property Component mapCanvasComponent
    property MapCanvas _canvas

    property var angle: 30
    property var offset: 10

    property var intersections_canvas: []
    property var centroid
    property var intersections_cartesian: []
    property var intersections_geographic: []

    property var px_multiplier

    property var closed: false
    property var detection_intersect: 0

    signal end_security

    Component.onCompleted: {
        Helper.init_lib(QtPositioning)
        mapCanvasComponent = Qt.createComponent("MapCanvas.qml");
        _canvas = mapCanvasComponent.createObject(map)
        map.addMapItem(_canvas)
    }

    function clear_path(){
        while(path.length > 0)
            removeCoordinate(0)
        closed = false
    }
    function map_check_intersections(pf){
        if (pathLength() < 3)
            return 0

        var pf_minus1 = map.fromCoordinate(coordinateAt(pathLength()-2))
        var mf = Helper.slope(pf_minus1,pf)

        var p_0 = map.fromCoordinate(coordinateAt(0))
        var m_0_f = Helper.slope(p_0,pf)

        detection_intersect = 0

        for(var i = 0; i < pathLength()-2; i++){
            var pi = map.fromCoordinate(path[i])
            var pi_plus1 = map.fromCoordinate(path[i+1])
            var mi = Helper.slope(pi_plus1, pi)


            //Check intersection between last line and the other segments
            var intersection_point = Helper.intersect_two_lines(pi, mi, pf, mf)
            if(intersection_point !== pf_minus1 &&
                intersection_point[2] !== 0 &&
                Helper.point_in_box(intersection_point,pf, pf_minus1) &&
                Helper.point_in_box(intersection_point,pi, pi_plus1)){
                //toast.show("INTERSECTION at segment"+(i+1))
                detection_intersect = 1
                return detection_intersect
            }

            //TODO -> Avoid the error happens in closing (when the line between first and last intersect others)

        }
        return detection_intersect
    }


    function create_JSON(){
        var security_path = {}
        security_path.name = 'SecurityPoly'
        security_path.values = []
        for(var j = 0; j < polysec_cur.pathLength(); j++){
            var p_i = polysec_cur.coordinateAt(j)
            security_path.values.push({
                  latitude: p_i.latitude,
                  longitude: p_i.longitude
              })
        }
        console.log(JSON.stringify(security_path))

        return security_path
    }


    function pos_changed_handler(mouse){
        if (polygonal_phase === 0) return;
        var p = Qt.point(mouse.x, mouse.y)
        var pf = map.toCoordinate(p)
        var last_idx = pathLength()-1
        var color = "#4ac7c0"

        //Check if the last point is near to the first one
        if((Helper.distance(map.fromCoordinate(coordinateAt(0)), p) < 15) || map_check_intersections(p)){
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
                polygonal_phase = 2
                addCoordinate(p)

                if(Helper.distance(p1, m) < 15 || detection_intersect){
                    removeCoordinate(pathLength()-1)
                    line.color = "#161fc4"
                    mapMouseArea.hoverEnabled = false
                    close_polygon()
                }

            }
        } else if (mouse.button & Qt.RightButton){
            close_polygon()
        }
    }

    function close_polygon(){
        if (polygonal_phase === 2){
            replaceCoordinate(pathLength()-1, coordinateAt(0))
            mapMouseArea.hoverEnabled = false
            line.color = "#161fc4"
            polygonal_phase = 0
            closed = true
            create_JSON()
            end()
        }
    }
}
