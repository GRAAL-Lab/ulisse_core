import QtQuick 2.0
import QtLocation 5.6

MapQuickItem{
    id: root
    property alias canvasWidth: _c.width
    property alias canvasHeight: _c.height
    property alias canvasCtx: _c._ctx
    property var canvasAngle: 0
    function requestPaint(){_c.requestPaint()}
    sourceItem: Canvas {
        id: _c
        antialiasing: true
        property var _ctx
        onAvailableChanged: {_ctx = _c.getContext("2d")}
        transform: [
            Rotation {
                origin.x: 0
                origin.y: 0
                axis.z: 1
                angle: root.canvasAngle
            }
        ]
    }
}
