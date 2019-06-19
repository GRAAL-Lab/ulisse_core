import QtLocation 5.6
import QtPositioning 5.6
import QtQuick 2.6
import QtQuick.Controls 2.1
import "../scripts/helper.js" as Helper

Item {

    property alias rulerTimer: scaleTimer
    //property bool followme: false
    property var scaleLengths: [5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 500000, 1000000, 2000000]

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

    PositionSource {
        id: positionSource
        active: false

        onPositionChanged: {
            map.center = positionSource.position.coordinate
        }
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

    Item {
        id: my_scale
        z: map.z + 3
        visible: scaleText.text != "0 m"
        anchors.bottom: parent.bottom
        anchors.right: parent.right
        anchors.margins: 15
        height: scaleText.height * 3
        width: scaleImage.width

        Image {
            id: scaleImageLeft
            source: "qrc:/images/scale_end.png"
            anchors.top: scaleText.bottom
            anchors.right: scaleImage.left
        }
        Image {
            id: scaleImage
            source: "qrc:/images/scale.png"
            anchors.top: scaleText.bottom
            anchors.right: scaleImageRight.left
        }
        Image {
            id: scaleImageRight
            source: "qrc:/images/scale_end.png"
            anchors.top: scaleText.bottom
            anchors.right: parent.right
        }
        Label {
            id: scaleText
            color: "#004EAE"
            anchors.centerIn: parent
            text: "0 m"
        }
        Component.onCompleted: {
            calculateScale()
        }
    }
}
