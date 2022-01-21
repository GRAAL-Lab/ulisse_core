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

    //property alias map: map

    Timer {
        // A Timer to center the catamaran in the screen when "Follow Vehicle"
        // option is enabled.
        id: followMeTimer
        interval: 250
        running: false
        repeat: true

        onTriggered: { map.center = fbkUpdater.ulisse_pos }
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

    engineButton.onClicked: {
        cmdWrapper.sendThrusterActivation(true);
    }

    followMeCheckbox.onCheckStateChanged: {
        followMeCheckbox.checked ? followMeTimer.start() : followMeTimer.stop()
    }


    Timer {
        // A Timer to center the catamaran in the screen at application startup
        id: centerOnStartTimer
        interval: 150
        running: true
        repeat: false

        onTriggered: {
            map.center = fbkUpdater.ulisse_pos
        }
    }
}
