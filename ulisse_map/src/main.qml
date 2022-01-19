import QtQuick 2.6
import QtQuick.Window 2.0
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtLocation 5.6
import QtPositioning 5.6
import Qt.labs.settings 1.0
import QtQuick.Dialogs 1.2
import QtQuick.Controls.Material 2.1
import QtQuick.Controls.Universal 2.1
import QtQuick.Controls.Styles 1.4
import "./qml"

ApplicationWindow {
    id: window
    minimumWidth: 1100
    minimumHeight: 800
    visible: true
    //visibility: "Maximized"

    property color blue: Material.color(Material.Blue, Material.Shade700)
    property color orange: Material.color(Material.Amber, Material.Shade700)
    property color softorange: Material.color(Material.DeepOrange, Material.Shade500)
    property color red: Material.color(Material.Red, Material.Shade700)
    property color lightred: Material.color(Material.Red, Material.Shade500)
    property color green: Material.color(Material.Green, Material.Shade700)
    property color lightgreen: Material.color(Material.Green, Material.Shade500)
    property color cyan: Material.color(Material.Cyan, Material.Shade700)
    property color lightcyan: Material.color(Material.Cyan, Material.Shade500)
    property color darkgrey: Material.color(Material.Grey, Material.Shade800)
    property color grey: Material.color(Material.Grey, Material.Shade700)
    property color lightgrey: Material.color(Material.Grey, Material.Shade400)
    property color lightergrey: Material.color(Material.Grey, Material.Shade100)

    property color mainColor: cyan
    property color mainColorLight: lightcyan

    property string futureMapPlugin: ""

    signal sig_escape

    Material.theme: settings.theme
    Material.accent: mainColor

    onClosing: {
        // This onClosing function is needed since the map
        // plugin cannot be changed 'live', so we will register
        // the new setting only when closing the app
        settings.mapPluginType = futureMapPlugin
    }

    Shortcut {
        // Halting catamaran when Return is pressed
        sequence: "Return"
        context: Qt.ApplicationShortcut
        onActivated: {
            toast.show("Sent Halt Command")
            cmdWrapper.sendHaltCommand()
        }
    }

    Shortcut {
        sequence: StandardKey.Cancel  // Escape Key
        context: Qt.ApplicationShortcut
        onActivated: {
            sig_escape()
        }
    }


    Settings {
        id: settings
        property int shTimeout: 120
        property string mapPluginType: "esri"
        property string esriMapCacheDir: home_dir + "/.map_offline_tiles/esri/"
        property string theme: "Light"
        property var mapBearing: 0.0
        property var mapZoom: 19.0
        property var mapCenter: QtPositioning.coordinate(44.392, 8.945)
        property string savedBoundary: "null"
        property bool showObstacleID: true
        property bool showCentroid: true
        property bool showStatusOverlay: true
        property string bagSaveFolder: home_dir + "/logs/"
        property int obstacleTimeout: 30

        Component.onCompleted: {
            futureMapPlugin = mapPluginType
            mapViewLoader.active = true

            /*var polygonComponent = Qt.createComponent("MapPolygon.qml")
            savedBoundary = polygonComponent.createObject()*/
        }
    }

    ToastManager {
        // The object that makes appear temporary messages on the window
        id: toast
        objectName: "toastManager"
    }

    SettingsDialog {
        id: settingsDialog
        x: Math.round((window.width - width) / 2)
        y: Math.round(window.height / 6)
        width: Math.round(Math.min(window.width, window.height) / 4 * 3)
    }

    HelpDialog {
        id: helpDialog
        x: Math.round((window.width - width) / 2)
        y: Math.round((window.height - height) / 2) - headerBar.height
        width: Math.round(window.width * 0.8)
        height: Math.round(window.height * 0.7)
    }

    FileDialog {
        id: browseCacheDirDialog
        title: "Please choose a folder"
        folder: shortcuts.home
        selectFolder: true

        onAccepted: {
            var path = browseCacheDirDialog.fileUrl.toString()
            // remove prefixed "file://"
            path = path.replace(/^(file:\/{2})/, "")
            // unescape html codes like '%23' for '#'
            var cleanPath = decodeURIComponent(path)
            settingsDialog.mapCacheDirText = cleanPath
        }
    }

    //// UI part ////
    header: CustomHeader {
        id: headerBar
    }

    Item {
        // This Item is needed to add margins to the StackLayout and make it correctly resize
        id: stackViewContainer
        anchors.fill: parent
        anchors.horizontalCenter: parent.horizontalCenter
        height: window.height - headerBar.height
        width: window.width

        Component {
            // This Component is needed to load the map in an asynchronous way,
            // using the Loader class.
            id: mapViewComponent
            MapView {
                id: mapView
                anchors.fill: parent
            }
        }

        StackLayout {
            id: mainStackView
            //height: parent.height
            anchors.fill: parent
            currentIndex: headerBar.tabBarIndex
            Layout.alignment: Qt.AlignHCenter
            Layout.fillHeight: true
            Layout.fillWidth: true

            Loader {
                // This loader is needed to dynamically load the map plugin
                // only once the settings are loaded (so to be able to
                // correctly read 'mapPluginType' and 'esriMapCacheDir').
                id: mapViewLoader
                sourceComponent: mapViewComponent
                active: false
                //Layout.fillHeight: true
                Layout.fillWidth: true
            }

            StatusDataView {
                id: statusDataView
                Layout.fillWidth: true
                Layout.fillHeight: true
            }

            ActionView {
                id: taskDataView
                Layout.fillWidth: true
                Layout.fillHeight: true
            }

            UtilitiesView {
                id: utilitiesView
                Layout.fillWidth: true
                Layout.fillHeight: true
            }
        }
    }
}
