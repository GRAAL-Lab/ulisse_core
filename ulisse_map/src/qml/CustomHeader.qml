import QtQuick 2.11
import QtQuick.Controls 2.4
import QtQuick.Controls.Material 2.4
import QtQuick.Controls.Universal 2.4
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.11
import QtQuick.Window 2.4
import QtLocation 5.11
import QtPositioning 5.11
import Qt.labs.settings 1.0
import QtGraphicalEffects 1.0
import "."

ToolBar {
    property alias tabBarIndex: headerLayout.tabBarIndex
    id: toolBar
    width: parent.width

    property bool multichoice: false

    Material.elevation: 4

    Material.foreground: lightgrey
    Material.background: (settings.theme == "Light" ? blue : lightred)
    Material.accent: "white"

    Layout.alignment: Qt.AlignLeft

    RowLayout {
        property alias tabBarIndex: headerBar.currentIndex
        id: headerLayout
        width: parent.width - toolButton.width
        spacing: 0

        TabBar {
            id: headerBar
            width: headerLayout.width
            TabButton {
                text: qsTr("Map")
                Layout.fillWidth: true
                width: headerLayout.width / 2
            }
            TabButton {
                text: qsTr("Data")
                Layout.fillWidth: true
                width: headerLayout.width / 2
            }
        }
    }

    ToolButton {
        id: toolButton
        anchors.right: parent.right
        text: qsTr("⋮")
        font.pointSize: 16
        font.weight: Font.Bold
        Material.background: grey

        onClicked: optionsMenu.open()
        Menu {
            id: optionsMenu
            x: parent.width - width
            y: parent.height
            transformOrigin: Menu.TopRight

            MenuItem {
                text: "Settings"
                onTriggered: {
                    settingsDialog.open()
                }
            }

            MenuItem {
                text: "Help"
                onTriggered: helpDialog.open()
            }

            MenuSeparator {
            }

            MenuItem {
                text: "Quit"
                onTriggered: Qt.quit()
            }
        }
    }
}
