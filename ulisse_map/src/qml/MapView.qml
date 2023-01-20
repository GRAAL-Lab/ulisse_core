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

    enum PolygonType {
        SafetyArea,
        Path
    }

    Timer {
        // A Timer to center the catamaran in the screen when "Follow Vehicle"
        // option is enabled.
        id: followMeTimer
        interval: 250
        running: false
        repeat: true

        onTriggered: {
            map.center = fbkUpdater.ulisse_pos
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

    enableRefButton.onClicked: {
        cmdWrapper.sendEnableReference(true);
    }

    enginePowerButton.onClicked: {
        cmdWrapper.toggleEnginePowerButtons();
    }



    followMeCheckbox.onCheckStateChanged: {
        followMeCheckbox.checked ? followMeTimer.start() : followMeTimer.stop()
    }


    Timer {
        // A Timer to perform post-startup operations
        id: centerOnStartTimer
        interval: 200
        running: true
        repeat: false

        onTriggered: {
            map.center = fbkUpdater.ulisse_pos
            //console.log("settings.unIndiceSalvatoACaso: " + settings.unIndiceSalvatoACaso)
            //console.log("settings.mapTypeIndex: " + settings.mapTypeIndex)
            //console.log("map.supportedMapTypes[settings]: " + map.supportedMapTypes[settings.mapTypeIndex])
            //console.log("map.activeMapType: " + map.activeMapType)

            map.activeMapType = map.supportedMapTypes[settings.mapTypeIndex]
        }
    }
}
