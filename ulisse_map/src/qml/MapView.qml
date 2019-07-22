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
    marker_coords: QtPositioning.coordinate(44.4, 8.94)

    Component.onCompleted: {
        console.log(("Current cache for ESRI Map plugin: %1").arg(mapCache.value))
    }

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
                          | (pathCurrentState
                             === pathState.empty) ? Material.color(
                                                        Material.DeepOrange,
                                                        Material.Shade600) : Material.color(
                                                        Material.Green,
                                                        Material.Shade500)
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
}
