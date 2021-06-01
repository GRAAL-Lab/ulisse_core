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
            font.pointSize: 26
        }

        transform: [
                Rotation {
                    origin.x: letter.width / 2
                    origin.y: letter.height / 2
                    angle: map.bearing
                }
            ]
    }
    anchorPoint.x: 20 //map_marker_letter.sourceItem.width / 2
    anchorPoint.y: 20 //map_marker_letter.sourceItem.height / 2

}
