import QtQuick 2.0
import QtLocation 5.6
import "."

MapQuickItem {
    id: root
    property alias canvasWidth: _c.width
    property alias canvasHeight: _c.height
    property alias canvasCtx: _c._ctx
    property var canvasAngle: 0
    property var multiplier: 0
    property alias _canvas: _c

    function requestPaint() {
        _c.requestPaint()
    }

    signal contextReady

    sourceItem: Canvas {
        id: _c
        //antialiasing:true
        //smooth: true
        property var _ctx

        onAvailableChanged: {
            // QML Canvas: Unable to use getContext() at this time, please wait for available: true
            _ctx = _c.getContext("2d")
            contextReady()
            //console.log("[Map Canvas] onAvailableChanged triggered")
        }

        transform: [
            Rotation {
                origin.x: 0
                origin.y: 0
                axis.z: 1
                angle: root.canvasAngle
            }
        ]
    }
    function clear_canvas() {
        canvasCtx.clearRect(0, 0, canvasWidth, canvasHeight)
        _c.requestPaint()
    }

}
