import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtLocation 5.6
import QtPositioning 5.6
import Qt.labs.settings 1.0
import QtQuick.Controls.Material 2.1
import QtQuick.Controls.Universal 2.1
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.2
import "."

MapViewForm {
    Timer {
        id: followMeTimer
        interval: 250
        running: false
        repeat: true

        onTriggered: {
            map.center = QtPositioning.coordinate(
                        fbkUpdater.ulisse_pos.latitude,
                        fbkUpdater.ulisse_pos.longitude)
        }
    }

    Component {
        id: mapCircleComponent
        MapCircle {
            radius: 10
            color: 'transparent'
            border.width: 2
            border.color: (pathCurrentState === pathState.creating)
                          | (pathCurrentState === pathState.empty) ? orange : lightgreen
            z: map.z + 1
        }
    }

    recenterButton.onClicked: {
        map.center = fbkUpdater.ulisse_pos
    }

    clearPathButton.onClicked: {
        map.clearUlisseTrace()
    }

    engine.onClicked: {
        cmdWrapper.sendThrusterActivation(true);
    }

    map.onZoomLevelChanged: {
        for (var i = 0; i < slidersLeft.columnTrack.children.length; i++) {
            slidersLeft.columnTrack.children[i].managed_path.a_marker.zoomLevel = map.zoomLevel / 2 + 9
            slidersLeft.columnTrack.children[i].managed_path.b_marker.zoomLevel = map.zoomLevel / 2 + 9
        }
    }
}
