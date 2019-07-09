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
    property var marker_coords
    property bool ulisse_state_changed: false
    property real myElevation: 6
    property real panesMargin: 14

    property int currentState: generalState.empty
    property int pathCurrentState: pathState.empty
    property var mapCircles: []
    property alias recenterButton: recenterButton
    property alias followMeCheckbox: followMeCheckbox
    property alias clearPathButton: clearPathButton
    property alias overlayStatusCbox: overlayStatusCbox
    property alias mapCache: mapCache
    property alias map: map
    property alias mapsidebar: mapsidebar
    property alias pathState: pathState
    property alias generalState: generalState
    property alias rectState: rectState
    spacing: 0

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
    }

    QtObject {
        id: generalState
        property int empty: 0
        property int path: 1
        property int rect: 2
        property int poly: 3
    }

    QtObject {
        id: pathState
        property int empty: 0
        property int creating: 1
        property int active: 2
        property int stopped: 3
    }
    QtObject {
        id: rectState
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
        Layout.minimumWidth: 400
        Layout.preferredWidth: 400
        Layout.maximumWidth: 400
    }

    property real my_height: 17

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
            height: parent.height - bottomToolbar.height
            plugin: mapPlugin
            center: marker_coords // Genoa
            zoomLevel: 17.5 //(maximumZoomLevel - minimumZoomLevel)/2
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
                    id: recenterButton
                    text: "Recenter"
                    highlighted: true
                    Material.accent: mainColor
                    Layout.leftMargin: 5
                }

                CheckBox {
                    id: followMeCheckbox
                    text: "Follow vehicle"
                    Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                    Material.accent: mainColor
                    checked: false
                }

                CheckBox {
                    id: overlayStatusCbox
                    text: "Show Overlay"
                    Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                    Material.accent: mainColor
                    checked: false
                }

                Button {
                    id: clearPathButton
                    Layout.rightMargin: 5
                    text: "Clear trace"
                    highlighted: true
                    Material.accent: mainAccentColor
                    Layout.alignment: Qt.AlignRight
                }
            }
        }
    }
}
