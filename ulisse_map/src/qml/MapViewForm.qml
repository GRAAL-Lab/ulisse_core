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
    property real myElevation: 0
    property real panesMargin: 14

    property int currentState: generalState.empty
    property int pathCurrentState: pathState.empty
    property var mapCircles: []
    property alias recenterButton: bottomToolbar.recenterButton
    property alias followMeCheckbox: bottomToolbar.followMeCheckbox
    //property alias overlayStatusCbox: bottomToolbar.overlayStatusCbox
    property alias clearPathButton: bottomToolbar.clearPathButton
    property alias gpsIconCBox: bottomToolbar.gpsIconCBox
    property alias engineButton: bottomToolbar.engineButton
    property alias mapCache: mapCache
    property alias map: map
    property alias mapsidebar: mapsidebar
    property alias pathState: pathState
    property alias pathCmdPane: mapsidebar.pathCmdPane



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

        //PluginParameter { name: "osm.useragent"; value: "GraalLab - UNIGE" }
        //PluginParameter { name: "osm.mapping.copyright"; value: "All mine" }

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


    MapSidebar {
        id: mapsidebar
        Layout.fillHeight: true
        Layout.minimumHeight: 150
        Layout.minimumWidth: 320
        Layout.preferredWidth: 320
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
            height: parent.height// - bottomToolbar.height
            plugin: mapPlugin
            center: settings.mapCenter
            zoomLevel: settings.mapZoom
            bearing: settings.mapBearing
            //activeMapType:
            //maximumZoomLevel: 20
            //style: Map.StreetMap

            onMapReadyChanged: {
                /*
                mapPluginReady()*/
                //console.log("activeMapType: " + activeMapType)
            }

        }

        Rectangle {
            id: pathManageToolbar
            color: Qt.rgba(0.9, 0.45, 0.1, 0.75)
            implicitWidth: bar_manage.implicitWidth
            implicitHeight: bar_manage.implicitHeight
            anchors.bottom: bottomToolbar.top
            anchors.right: parent.right
            anchors.left: parent.left

            RowLayout {
                anchors.fill: parent
                BarManagePaths {
                    id: bar_manage
                    height: 130
                    Layout.fillWidth: true
                    Layout.alignment: Qt.AlignHCenter | Qt.AlignBottom
                    Layout.bottomMargin: 10
                }
            }
        }

        BottomToolbar {
            id: bottomToolbar
            width: parent.width
            height: clearPathButton.height
            color: Material.background
            anchors.bottom: parent.bottom
        }
    }
}

