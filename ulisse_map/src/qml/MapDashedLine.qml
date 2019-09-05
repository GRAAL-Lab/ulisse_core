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

MapPolyline {
    id: root
    line.color: red
    opacity: 1
    z: map.z + 2

    Component.onCompleted: function () {
        addCoordinate(QtPositioning.coordinate(0, 0))
        addCoordinate(QtPositioning.coordinate(0, 0))
    }

    function reset() {
        replaceCoordinate(0, QtPositioning.coordinate(0, 0))
        replaceCoordinate(1, QtPositioning.coordinate(0, 0))
    }
}
