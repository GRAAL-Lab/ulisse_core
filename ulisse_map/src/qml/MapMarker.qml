import QtQuick 2.0
import QtLocation 5.6
import QtPositioning 5.6
import "."

MapCircle {
    id: root
    radius: 5
    color: red
    border.width: 1
    border.color: grey
    opacity: 0
    z: map.z + 2
}
