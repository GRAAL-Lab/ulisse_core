import QtQuick 2.6
import QtQuick.Window 2.0
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtLocation 5.6
import QtPositioning 5.6
import Qt.labs.settings 1.0
import QtQuick.Controls.Material 2.1
import QtQuick.Controls.Universal 2.1
import QtQuick.Controls.Styles 1.4
import "./qml"

ApplicationWindow {
    id: window
    width: 800
    height: 640
    visible: true

    minimumHeight: 200
    minimumWidth: 300

    property var mainColor: Material.Cyan

    Material.theme: Material.Light
    Material.accent: mainColor

    /* Halting catamaran when space is pressed */
    Shortcut {
        sequence: " "
        onActivated: cmdWrapper.sendHaltCommand()
    }

    Settings {
        id: settings
        property string style: "Material"
    }

    header: CustomHeader {
        id: headerBar
    }

    Item{
        // This Item is needed to add margins to the StackLayout and make it correctly resize
        id: stackViewContainer
        anchors.fill: parent
        anchors.horizontalCenter: parent.horizontalCenter;
        height: window.height - headerBar.height// - customFoot.height
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
                id: mapview
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
}
