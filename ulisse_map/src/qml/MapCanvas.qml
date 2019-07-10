import QtQuick 2.0
import QtLocation 5.6

MapQuickItem{
    property alias canvasWidth: _c.width
    property alias canvasHeight: _c.height
    property alias canvasAngle: _c.rotation
    property alias canvasCtx: _c._ctx
    function requestPaint(){_c.requestPaint()}
    sourceItem: Canvas {
        id: _c
        antialiasing: true
        property var _ctx
        onAvailableChanged: {
            _ctx = _c.getContext("2d")
        }
    }
}
