import QtLocation 5.6
import QtPositioning 5.6
import QtQuick 2.6
import QtQuick.Controls 2.1
import "."
import "../scripts/helper.js" as Helper

Item {
    property alias my_scale: my_scale
    property alias positionSource: positionSource
    property alias scaleImage: scaleImage
    property alias scaleText: scaleText
    property alias scaleImageLeft: scaleImageLeft
    property var scaleLengths: [5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 500000, 1000000, 2000000]
    width: 50
    height: 50

    PositionSource {
        id: positionSource
        active: false
    }

    Item {
        id: my_scale
        z: map.z + 10
        visible: scaleText.text != "0 m"
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
    }
    states: [
        State {
            name: "State1"
            PropertyChanges {
                target: scaleText
                text: "1 m"
            }
        }
    ]
}
