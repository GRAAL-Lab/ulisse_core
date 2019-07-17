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

    Component.onCompleted: {
        Helper.init_lib(QtPositioning)
        mapCanvasComponent = Qt.createComponent("MapCanvas.qml");
        _canvas = mapCanvasComponent.createObject(map)
        map.addMapItem(_canvas)
    }

    //TODO
    function map_check_intersections(pc){
        var i
        var last_point = map.fromCoordinate(coordinateAt(pathLength()));

        for (i = 0; i < pathLength() - 2; i ++){

            var pa = map.fromCoordinate(coordinateAt(i))
            var pb = map.fromCoordinate(coordinateAt(i+1))

            //var intersections = Helper.intersections(last_point, )
           // if()
        }
    }


    //TODO
    //console.log(JSON.stringify())
    function create_JSON(){
        var json_data = '{"points":';

        var i;
        for(i = 0; i < pathLength(); i++){
            var p_i = map.fromCoordinate(coordinateAt(i))
            json_data += '["latitude":'+p_i.x+',"longitude":'+p_i.y+']';
        }
        json_data += '}';

        console.log(JSON.stringify(json_data));
    }

    function pos_changed_handler(mouse){
        if (polygonal_phase === 0) return;
        var p = Qt.point(mouse.x, mouse.y)
        var pf = map.toCoordinate(Qt.point(mouse.x, mouse.y))
        var last_idx = pathLength()-1
        var color = "#4ac7c0"
        if (polygonal_phase === 3){
            pf = coordinateAt(0)
            color = "#4ac7c0"
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
                polygonal_phase = 2
                addCoordinate(p)
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
