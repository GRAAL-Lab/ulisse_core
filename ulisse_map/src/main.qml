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
    width: 900
    height: 760
    visible: true

    minimumHeight: 500
    minimumWidth: 705

    property var mainColor: (settings.theme == "Light" ? Material.Cyan : Material.Red)
    property var mainAccentColor: Material.color(Material.Amber, Material.Shade700)
    property var secondaryAccentColor: Material.color(Material.Green, Material.Shade600)

    Material.theme: settings.theme
    Material.accent: mainColor


    /* Halting catamaran when space is pressed */
    Shortcut {
        sequence: " "
        onActivated: {
            toast.show("Sent Halt Command")
            cmdWrapper.sendHaltCommand()
        }
    }

    Settings {
        id: settings
        property string theme: "Light"
    }

    header: CustomHeader {
        id: headerBar
    }

    Item{
        // This Item is needed to add margins to the StackLayout and make it correctly resize
        id: stackViewContainer
        anchors.fill: parent
        anchors.horizontalCenter: parent.horizontalCenter;
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

            MapView {
                id: mapView
                anchors.fill: parent
            }

            DataView {
                id: dataView
                anchors.fill: parent
                anchors.margins: 10
            }
        }
    }

    ToastManager {
        id: toast
        objectName: "toastManager"
    }

    SettingsDialog {
        id: settingsDialog
        x: Math.round((window.width - width) / 2)
        y: Math.round(window.height / 6)
        width: Math.round(Math.min(window.width, window.height) / 3 * 2)
    }

    HelpDialog {
        id: helpDialog
        x: Math.round((window.width - width) / 2)
        y: Math.round((window.height - height) / 2) - headerBar.height
        width: Math.round(window.width * 0.8)
        height: Math.round(window.height * 0.7)

    }

    FileDialog {
        id: savePathDialog
        title: "Please choose a file"
        folder: shortcuts.home
        selectExisting: false

        onAccepted: {
            console.log("You chose: " + savePathDialog.fileUrls)
            cmdWrapper.savePathToFile(savePathDialog.fileUrls);
        }


    }

    FileDialog {
        id: loadPathDialog
        title: "Please choose a file"
        folder: shortcuts.home
        nameFilters: ["Path Files (*.path)"]

        onAccepted: {
            //console.log("You chose: " + loadPathDialog.fileUrls)
            cmdWrapper.loadPathFromFile(loadPathDialog.fileUrls);
        }
    }


    /*Text {
        color: "#2b2b2b"
        z: 10
        anchors.centerIn: parent
        //anchors.fill: parent
        text: "%1 x %2".arg(window.width).arg(window.height)
        font.family: "Ubuntu Mono"
    }*/

}
