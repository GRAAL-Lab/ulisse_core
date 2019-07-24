import QtQuick 2.0
import QtLocation 5.6
import QtPositioning 5.6

MapQuickItem {

    property alias source: img.source
    id: root
    opacity: 0
    z: map.z + 2
    sourceItem: Item {
        width: 40
        height: 40
        Image {
            id: img
            width: 40
            height: 40
        }
    }
    anchorPoint.x: 20
    anchorPoint.y: 20
}
