import QtQuick 2.0
import QtLocation 5.6
import QtPositioning 5.6

MapQuickItem {
    id: map_marker_letter
    property alias content: letter.text

    opacity: 0
    z: map.z + 5
    sourceItem: Item {
        Text {
            id: letter
            text: ""
            font.family: "Helvetica"
            font.pointSize: 20
        }

        Component.onCompleted: {
            map_marker_letter.anchorPoint.x = 0
            map_marker_letter.anchorPoint.y = 5 // letter.height / 4
        }

        transform: [
                Rotation {
                    origin.x: letter.width / 2
                    origin.y: letter.height / 2
                    angle: map.bearing
                }
            ]
    }



}
