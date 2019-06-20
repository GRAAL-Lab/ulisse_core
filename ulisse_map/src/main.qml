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
    width: 940
    height: 720
    visible: true

    minimumHeight: 500
    minimumWidth: 705

    property var mainColor: (settings.theme == "Light" ? Material.Cyan : Material.Red)
    property var mainAccentColor: Material.color(Material.Amber,
                                                 Material.Shade700)
    property var secondaryAccentColor: Material.color(Material.Green,
                                                      Material.Shade600)
    property string futureMapPlugin: ""

    Material.theme: settings.theme
    Material.accent: mainColor

    onClosing: {
        // This onClosing function is needed since the map
        // plugin cannot be changed 'live', so we will register
        // the new setting only when closing the app
        settings.mapPluginType = futureMapPlugin
    }

    Shortcut {
        // Halting catamaran when space is pressed
        sequence: " "
        onActivated: {
            toast.show("Sent Halt Command")
            cmdWrapper.sendHaltCommand()
        }
    }

    Settings {
        id: settings
        property int shTimeout: 120
        property string mapPluginType: "esri"
        //FIXME: hard path
        property string esriMapCacheDir: home_dir + "/.map_offline_tiles/esri/"
        property string theme: "Light"

        Component.onCompleted: {
            futureMapPlugin = mapPluginType
            mapViewLoader.active = true
        }
    }

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

        StackLayout {
            id: mainStackView
            height: parent.height
            anchors.fill: parent
            currentIndex: headerBar.tabBarIndex
            Layout.alignment: Qt.AlignHCenter
            Layout.fillHeight: true
            Layout.fillWidth: true

            Loader {
                // This loader is need to dynamically load the map plugin
                // only once the settings are loaded (so to be able to
                // correctly read 'mapPluginType' and 'esriMapCacheDir').
                id: mapViewLoader
                sourceComponent: mapViewComponent
                active: false
                Layout.fillHeight: true
                Layout.fillWidth: true
            }

            DataView {
                id: dataView
                Layout.fillHeight: true
                Layout.fillWidth: true
                Layout.margins: 10
            }
        }
    }

    Component {
        id: mapViewComponent
        MapView {
            id: mapView
            anchors.fill: parent
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

    /*Text {
        id: loadingText
        //color: "#2b2b2b"
        z: 10
        anchors.centerIn: parent
        //anchors.fill: parent
        color: 'white'
        text: "Loading... (%1 x %2)".arg(window.width).arg(window.height)
        font.family: "Ubuntu Mono"
    }*/
}
