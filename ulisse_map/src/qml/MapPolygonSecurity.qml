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

    signal end_security

    Component.onCompleted: {
        Helper.init_lib(QtPositioning)
        mapCanvasComponent = Qt.createComponent("MapCanvas.qml");
        _canvas = mapCanvasComponent.createObject(map)
        map.addMapItem(_canvas)
    }


    function map_check_intersections(){
        if (pathLength() < 3)
            return 0

        var pf = map.fromCoordinate(coordinateAt(pathLength()-2))
        var pf_minus1 = map.fromCoordinate(coordinateAt(pathLength()-3))
        var mf = (pf_minus1.y - pf.y)/(pf_minus1.x - pf.x)

        for(var i = 0; i < pathLength()-2; i++){
            var pi = map.fromCoordinate(coordinateAt(i))
            var pi_plus1 = map.fromCoordinate(coordinateAt(i+1))
            var mi = (pi_plus1.y - pi.y)/(pi_plus1.x - pi.x)

            var intersection_point = Helper.intersect_two_lines(pi, mi, pf, mf )

            if(intersection_point !== pf_minus1 && intersection_point[2] !== 0 &&
               Helper.point_in_box(intersection_point,pf, pf_minus1) && Helper.point_in_box(intersection_point,pi, pi_plus1)){

                toast.show("INTERSECTION at segment"+(i+1))
                return 1
            }
        }
        return 0
    }


    //TODO -> send it
    function create_JSON_OLD_ONE(){
        var json_data = '{"points":[';

        var i;
        //The last one is equals to the first one.
        //TODO -> mayabe add something like "number of points"
        for(i = 0; i < pathLength(); i++){
            var p_i = coordinateAt(i)
            json_data += '{"latitude":'+p_i.latitude+',"longitude":'+p_i.longitude+'}';
            if(i < pathLength()-1)
                json_data += ',';
        }
        json_data += ']}';
        //console.log(JSON.stringify(json_data));
    }

    function create_JSON(){
        var j
        var l
        var security_poly = {}
        security_poly.name = 'SecurityPoly'
        security_poly.values = []
        for(j = 0; j < polysec_cur.pathLength(); j++){
            var p_i = polysec_cur.coordinateAt(j)
            l = []
            l.push("latitude:"+p_i.latitude)
            l.push("longitude:"+p_i.longitude)
            security_poly.values.push(l)

        }
        //console.log(JSON.stringify(json_data));
    }


    function distance(p1,p2){
        return Math.sqrt(Math.pow(p1.x-p2.x,2) + Math.pow(p1.y-p2.y,2))
    }

    function pos_changed_handler(mouse){
        if (polygonal_phase === 0) return;
        var p = Qt.point(mouse.x, mouse.y)
        var pf = map.toCoordinate(Qt.point(mouse.x, mouse.y))
        var last_idx = pathLength()-1
        var color = "#4ac7c0"

        //Check if the last point is near to the first one
        if(distance(map.fromCoordinate(coordinateAt(0)), p) < 15){
            pf = coordinateAt(0)
            color = "#ffb300"
        }

        if(map_check_intersections()){
            removeCoordinate(pathLength()-1)
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
                //poligonal_direction = Helper.three_point_direction(p1,p2,p3)
                polygonal_phase = 2
                addCoordinate(p)

                if(distance(p1, p3) < 15){
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
            end()

            create_JSON()
        }
    }
}
