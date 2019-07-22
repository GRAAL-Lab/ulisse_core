import QtQuick 2.0
import QtLocation 5.6
import QtPositioning 5.6

import "../scripts/helper.js" as Helper

MapQuickItem{
    id: root
    property var h_center: QtPositioning.coordinate(0,0)
    property var h_handle: QtPositioning.coordinate(0,0)
    property var h_radius: 6
    function handle_select(yes){
        _handle.color = yes ? "#00ffff" : "#00ff00"
    }

    function center_select(yes){
        _center.color = yes ? "#00ffff" : "#0000ff"
    }

    function add_to_map(map){
        map.addMapItem(_handle)
        map.addMapItem(_center)
        map.addMapItem(_line)
        map.addMapItem(_canvas)
    }

    function deregister_map_items(){
        map.removeMapItem(_handle)
        map.removeMapItem(_center)
        map.removeMapItem(_line)
        map.removeMapItem(_canvas)
    }

    coordinate: h_center
    sourceItem: Item {
        z: map.z + 2
        MapCircle{
            id: _handle
            center: h_handle
            radius: h_radius
            color: "#00ff00"
            border.width: 1
            border.color: 'gray'
            opacity: root.opacity
        }

        MapPolyline{
            id: _line
            opacity: root.opacity
        }

        MapCircle{
            id: _center
            center: h_center
            radius: h_radius
            color: "#0000ff"
            border.width: 1
            border.color: 'gray'
            opacity: root.opacity
        }

        Canvas{
            id: _canvas
            property var _ctx
            antialiasing: true
            width: 80
            height: 80
            x: parent.x - 40
            y: parent.y - 40
            onAvailableChanged: {_ctx = getContext("2d")}
            opacity: root.opacity
        }
        opacity: root.opacity
    }

    Component.onCompleted: function(){
        _line.addCoordinate(QtPositioning.coordinate(0,0))
        _line.addCoordinate(QtPositioning.coordinate(0,0))
    }
    onH_centerChanged: function(){
        _line.replaceCoordinate(0, h_center)
    }
    onH_handleChanged: function(){
        _line.replaceCoordinate(1, h_handle)
    }
    property var cumulativeAngle: 0

    function update_canvas(degrees){
        cumulativeAngle+=degrees
        var ctx = _canvas._ctx
        if (ctx === null || ctx === undefined) return
        ctx.clearRect(0, 0, 80, 80)
        ctx.strokeStyle = "#000000"
        ctx.lineWidth = 1
        ctx.lineJoin = "bevel"
        ctx.beginPath()
        ctx.moveTo(40,40)
        ctx.lineTo(5,40)
        ctx.arc(40,
               40,
               35,
               Math.PI,
               Math.PI + Helper.deg_to_rad(cumulativeAngle),
               false)
        ctx.lineTo(40,40)
        ctx.stroke()
        _canvas.requestPaint()
    }
}
