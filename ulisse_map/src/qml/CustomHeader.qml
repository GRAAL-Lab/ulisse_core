import QtQuick 2.6
import QtQuick.Window 2.2
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.1
import QtQuick.Controls.Universal 2.1
import Qt.labs.settings 1.0
import QtQuick.Layouts 1.3
import QtGraphicalEffects 1.0
import "."

ToolBar {
    id: toolBar

    property alias tabBarIndex: headerLayout.tabBarIndex
    //property bool multichoice: false

    Material.foreground: dimmedwhite
    Material.background: mainColor
    Material.accent: "white"
    Material.elevation: 3
    Layout.alignment: Qt.AlignLeft
    width: parent.width

    RowLayout {
        property alias tabBarIndex: headerBar.currentIndex
        id: headerLayout
        width: parent.width - toolButton.width
        height: parent.height
        spacing: 0

        TabBar {
            id: headerBar
            width: headerLayout.width
            height: headerLayout.height
            Layout.topMargin: 2
            Layout.alignment: Qt.AlignTop

            TabButton {
                text: qsTr("Map")
                height: headerLayout.height
                width: headerLayout.width / 4

                background: Rectangle {
                    color: (headerBar.currentIndex == 0) ? mainColorLight : mainColor
                }
            }

            TabButton {
                text: qsTr("Status Data")
                height: headerLayout.height
                width: headerLayout.width / 4

                background: Rectangle {
                    color: (headerBar.currentIndex == 1) ? mainColorLight : mainColor
                }
            }

            TabButton {
                text: qsTr("Action View")
                height: headerLayout.height
                width: headerLayout.width / 4

                background: Rectangle {
                    color: (headerBar.currentIndex == 2) ? mainColorLight : mainColor
                }
            }

            TabButton {
                text: qsTr("Utilities")
                height: headerLayout.height
                width: headerLayout.width / 4

                background: Rectangle {
                    color: (headerBar.currentIndex == 3) ? mainColorLight : mainColor
                }
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
        Material.accent: "white"
        height: headerLayout.height

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
