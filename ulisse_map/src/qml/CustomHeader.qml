import QtQuick 2.6
import QtQuick.Window 2.2
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.1
import QtQuick.Controls.Universal 2.1
import Qt.labs.settings 1.0
import QtQuick.Layouts 1.3
import QtGraphicalEffects 1.0

ToolBar {
    property alias tabBarIndex: headerLayout.tabBarIndex
    id: toolBar
    width: parent.width
    Material.elevation: 4

    Material.foreground: Material.color(Material.Grey, Material.Shade300)
    Material.background: (settings.theme == "Light" ? Material.Cyan : Material.color(
                                                          Material.Red,
                                                          Material.Shade600))
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
        Material.background: Material.color(Material.Cyan, Material.Shade500)

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
