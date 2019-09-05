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
import "../scripts/helper.js" as Helper

MapRulerForm {
    property alias rulerTimer: scaleTimer

    function calculateScale() {
        var coord1, coord2, dist, text, f
        f = 0
        coord1 = map.toCoordinate(Qt.point(0, my_scale.y))
        coord2 = map.toCoordinate(Qt.point(0 + scaleImage.sourceSize.width,
                                           my_scale.y))
        dist = Math.round(coord1.distanceTo(coord2))

        if (dist === 0) {


            // not visible
        } else {
            for (var i = 0; i < scaleLengths.length - 1; i++) {
                if (dist < (scaleLengths[i] + scaleLengths[i + 1]) / 2) {
                    f = scaleLengths[i] / dist
                    dist = scaleLengths[i]
                    break
                }
            }
            if (f === 0) {
                f = dist / scaleLengths[i]
                dist = scaleLengths[i]
            }
        }

        text = Helper.formatDistance(dist)
        scaleImage.width = (scaleImage.sourceSize.width * f) - 2 * scaleImageLeft.sourceSize.width
        scaleText.text = text
    }

    positionSource.onPositionChanged: {
        map.center = positionSource.position.coordinate
    }

    Timer {
        id: scaleTimer
        interval: 100
        running: false
        repeat: false
        onTriggered: {
            calculateScale()
        }
    }

    Component.onCompleted: {
        calculateScale()
    }
}
