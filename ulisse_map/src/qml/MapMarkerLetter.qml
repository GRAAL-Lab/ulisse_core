import QtQuick 2.0
import QtLocation 5.6
import QtPositioning 5.6

MapQuickItem {
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
    }
    anchorPoint.x: 20
    anchorPoint.y: 20
}
