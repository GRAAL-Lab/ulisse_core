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
    overlayStatusCbox.onCheckStateChanged: {
        if (checked === true) {
            followMeTimer.start()
        } else {
            followMeTimer.stop()
        }
    }

    clearPathButton.onClicked: {
        map.clearUlisseTrace()
    }

    map.onZoomLevelChanged: {
        for (var i = 0; i < slidersLeft.columnTrack.children.length; i++) {
            slidersLeft.columnTrack.children[i].managed_path.a_marker.zoomLevel = map.zoomLevel / 2 + 9
            slidersLeft.columnTrack.children[i].managed_path.b_marker.zoomLevel = map.zoomLevel / 2 + 9
        }
    }
}
