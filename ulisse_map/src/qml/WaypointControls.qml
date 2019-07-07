import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import QtQuick.Dialogs 1.2

WaypointControlsForm {
    id: my_waypoint_ctrl
    waypointsButton.onClicked: {
            if (waypointRadius.text !== '') {
                if (mapView.pathCurrentState === pathState.empty) {
                    map.createPath()
                } else if (mapView.pathCurrentState === pathState.creating) {
                    map.startPath()
                } else if (mapView.pathCurrentState === pathState.active) {
                    map.stopPath()
                } else if (mapView.pathCurrentState === pathState.stopped) {
                    map.resumePath()
                }
            } else {
                acceptRadDialog.open()
            }
        }
    squaredrawButton.onClicked: {
                map.createRect()
        }
    wpRestartButton.onClicked: {
         cmdWrapper.startPath()
    }

    wpDeleteButton.onClicked: {
        map.deletePath()
    }
}


