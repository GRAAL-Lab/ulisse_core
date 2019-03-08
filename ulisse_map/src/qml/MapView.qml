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

RowLayout {
    spacing: 0
    property var marker_coords: QtPositioning.coordinate(44.4, 8.94)
    property bool ulisse_state_changed: false
    property real myElevation: 6
    property real panesMargin: 14

    property int pathCurrentState: pathState.empty
    property var mapCircles: []

    Plugin {
        id: mapPlugin
        name: settings.mapPluginType

        PluginParameter {
            id: mapCache
            name: "esri.mapping.cache.directory"
            value: settings.esriMapCacheDir
        }

        PluginParameter {
            name: "esri.mapping.maximumZoomLevel"
            value: 19.9
        }

        Component.onCompleted: {
            console.log(("Current cache for ESRI Map plugin: %1").arg(mapCache.value))
        }

    }

    QtObject {
        id: pathState
        property int empty: 0
        property int creating: 1
        property int active: 2
        property int stopped: 3
    }


    ModalPopup {
        id: acceptRadDialog
        dialogTitle: "Insert an acceptance radius"

    }

    ModalPopup {
        id: speedHeadingDialog
        dialogTitle: "Insert both speed (m/s) and heading (deg)"
    }


    MapSidebar {
        id: mapsidebar
        Layout.fillHeight: true
        Layout.minimumHeight: 150
        Layout.preferredWidth: 265
        Layout.maximumWidth: 265
        Layout.topMargin: 5
    }

    property real altezzaScrittaDemmerda: 17

    Rectangle {
        id: mapContainer
        Layout.fillWidth: true
        Layout.fillHeight: true
        Layout.minimumWidth: 300
        Layout.preferredWidth: 500
        Layout.preferredHeight: 500

        MapComponent {
            id: map
            width: parent.width
            height: parent.height - bottomToolbar.height + altezzaScrittaDemmerda
            plugin: mapPlugin
            center: QtPositioning.coordinate(44.393, 8.945) // Genoa
            zoomLevel: 17.5//(maximumZoomLevel - minimumZoomLevel)/2
        }

        Rectangle {
            id: bottomToolbar
            width: parent.width
            height: clearPathButton.height
            color: Material.background
            anchors.bottom: parent.bottom

            RowLayout {
                width: parent.width

                Button {
                    id:recenterButton
                    text: "Recenter"
                    highlighted: true
                    Material.accent: mainColor
                    Layout.leftMargin: 5
                    onClicked: {
                        map.center = ulisseIcon.coordinate;
                    }
                }

                CheckBox {
                    id: followMeCheckbox
                    text: "Follow vehicle"
                    anchors.left: recenterButton.right
                    Material.accent: mainColor
                    checked: false

                    Timer {
                        id: followMeTimer
                        interval: 250; running: false; repeat: true

                        onTriggered: {
                            map.center = ulisseIcon.coordinate;
                        }
                    }

                    onCheckStateChanged: {
                        if (checked === true){
                            followMeTimer.start()
                        } else {
                            followMeTimer.stop()
                        }
                    }

                }

                CheckBox {
                    id: overlayStatusCbox
                    text: "Show Overlay"
                    anchors.left: followMeCheckbox.right
                    Material.accent: mainColor
                    checked: false

                    onCheckStateChanged: {
                        if (checked === true){
                            overlayText.opacity = 1.0;
                        } else {
                            overlayText.opacity = 0.0;
                        }
                    }

                }

                Button {
                    id:clearPathButton
                    Layout.rightMargin: 5
                    text: "Clear trace"
                    highlighted: true
                    Material.accent: mainAccentColor
                    Layout.alignment: Qt.AlignRight

                    onClicked: {
                        ulissePath.path = [];
                        ulissePath.firstRun = true;
                    }
                }
            }
        }
    }

    Component {
        id: mapCircleComponent
        MapCircle {
            radius: mapsidebar.waypointRadius
            color: 'transparent'
            border.width: 2
            border.color: (pathCurrentState === pathState.creating) | (pathCurrentState === pathState.empty) ? Material.color(Material.DeepOrange, Material.Shade600) : Material.color(Material.Green, Material.Shade500)
            z: map.z + 1
        }
    }
}


