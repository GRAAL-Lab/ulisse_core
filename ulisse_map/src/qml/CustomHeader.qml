import QtQuick 2.6
import QtQuick.Window 2.2
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.1
import QtQuick.Controls.Universal 2.1
import Qt.labs.settings 1.0
import QtQuick.Controls.Styles 1.4
import QtQuick.Layouts 1.3
import QtGraphicalEffects 1.0

ToolBar {
    property alias tabBarIndex: headerBar.currentIndex
    width: parent.width
    Material.elevation: 2

    Material.foreground: Material.color(Material.Grey,Material.Shade300)//"white"
    Material.accent: "white"//Material.color(Material.BlueGrey,Material.Shade900)
    Material.background: Material.color(Material.Cyan,Material.Shade400)

    Layout.alignment: Qt.AlignLeft

    TabBar {
        id: headerBar
        anchors.left: parent.left//robotLogo.right
        width: parent.width - toolButton.width
        TabButton {
            text: qsTr("Map")
        }
        TabButton {
            text: qsTr("Data")
        }
        /*TabButton {
            text: qsTr("Graphs")
        }*/
    }

    ToolButton {
        id: toolButton
        anchors.right: parent.right
        contentItem: Image {
            id: menuIcon
            fillMode: Image.Pad
            horizontalAlignment: Image.AlignHCenter
            verticalAlignment: Image.AlignVCenter
            source: 'qrc:/images/menu.png'

            ColorOverlay {
                anchors.fill: menuIcon
                source: menuIcon
                color: (settings.style == "Material") ? "white" : "transparent"
            }
        }
        onClicked: optionsMenu.open()

        Menu {
            id: optionsMenu
            x: parent.width - width
            transformOrigin: Menu.TopRight

            /*MenuItem {
                text: "Settings"
                onTriggered: settingsDialog.open()
            }*/
            MenuItem {
                text: "Quit"
                onTriggered: Qt.quit()
            }
        }
    }
}
