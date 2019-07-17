import QtQuick 2.0
import QtLocation 5.6
import QtPositioning 5.6

MapPolyline {
    id: root
    line.color: "#ff0000"
    opacity: 1
    z: map.z + 2
}
