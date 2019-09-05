import QtQuick 2.11
import QtQuick.Controls 2.4
import QtQuick.Controls.Material 2.4
import QtQuick.Controls.Universal 2.4
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.11
import QtQuick.Window 2.4
import QtLocation 5.11
import QtPositioning 5.11
import Qt.labs.settings 1.0
import QtGraphicalEffects 1.0
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
    signal ctx_ready
    sourceItem: Canvas {
        id: _c
        antialiasing: true
        property var _ctx
        onAvailableChanged: {
            _ctx = _c.getContext("2d")
            ctx_ready()
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
