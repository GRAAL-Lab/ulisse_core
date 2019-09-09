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
    property bool ulisse_state_changed: false
    property real myElevation: 6
    property real panesMargin: 14

    property int currentState: generalState.empty
    property int pathCurrentState: pathState.empty
    property var mapCircles: []
    property alias recenterButton: recenterButton
    property alias followMeCheckbox: followMeCheckbox
    property alias clearPathButton: clearPathButton
    property alias mapCache: mapCache
    property alias map: map
    property alias mapsidebar: mapsidebar
    property alias pathState: pathState
    property alias slidersLeft: sidebar_manage

    spacing: 0
    width: window.width
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
        property int polysec: 4
        property int editmode: 5
        property int deletemode: 6
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
        Layout.minimumWidth: 300
        Layout.preferredWidth: 300
        Layout.maximumWidth: 350
        Material.elevation: myElevation
        Material.accent: grey
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

        SidebarManage {
            id: sidebar_manage
            anchors.bottom: bottomToolbar.top
            anchors.bottomMargin: 20
            edge: Qt.LeftEdge
        }

        Rectangle {
            id: manageToolbar
            height: bar_manage.height
            color: Material.background
            anchors.bottom: bottomToolbar.top
            anchors.left: parent.left
            anchors.right: parent.right
            RowLayout {
                anchors.fill: parent

                BarManagePaths {
                    id: bar_manage
                    width: parent.width
                    height: parent.height
                    Layout.alignment: Qt.AlignHCenter | Qt.AlignBottom
                    Layout.bottomMargin: 10
                }
            }
        }

        Rectangle {
            id: bottomToolbar
            width: parent.width
            height: clearPathButton.height
            color: Material.background
            anchors.bottom: parent.bottom

            RowLayout {
                anchors.fill: parent
                width: parent.width
                height: parent.height - recenterButton.height
                spacing: 2

                Button {
                    id: recenterButton
                    text: "Recenter"
                    highlighted: true
                    Material.background: blue
                    Layout.leftMargin: 10
                }

                Button {
                    id: clearPathButton
                    anchors.rightMargin: parent.anchors.rightMargin
                    text: "Clear trace"
                    highlighted: true
                    Material.accent: orange
                    Layout.alignment: Qt.AlignLeft
                    anchors.left: recenterButton.right
                    anchors.leftMargin: 10
                }
                CheckBox {
                    id: followMeCheckbox
                    text: "Follow vehicle"
                    Layout.alignment: Qt.AlignLeft
                    anchors.left: clearPathButton.right
                    Material.accent: mainColor
                    checked: false
                    anchors.leftMargin: 10
                }
                Button {
                    id: engine
                    anchors.rightMargin: parent.anchors.rightMargin
                    //Layout.rightMargin: 5
                    text: "engine"
                    highlighted: true
                    Material.accent: red
                    Layout.alignment: Qt.AlignRight
                    Layout.rightMargin: 20
                }
            }
        }
    }
}

/*##^##
Designer {
    D{i:0;height:800;width:1000}
}
##^##*/
