import QtQuick 2.0
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
MapPolyline {
    id: root
    line.color: Material.color(Material.Red, Material.Shade800)
    opacity: 1
    z: map.z + 2
    Component.onCompleted: function (){
        addCoordinate(QtPositioning.coordinate(0,0))
        addCoordinate(QtPositioning.coordinate(0,0))
    }

    function reset(){
        replaceCoordinate(0,QtPositioning.coordinate(0,0))
        replaceCoordinate(1,QtPositioning.coordinate(0,0))
    }
}
