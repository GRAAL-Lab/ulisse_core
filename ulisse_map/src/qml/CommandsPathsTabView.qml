import QtQuick 2.6
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls.Material 2.1

import "."
import "../scripts/helper.js" as Helper

Pane{
    property alias pathCommandsPane: pathCommandsPane

    topPadding: 0

    Material.elevation: 4

    ColumnLayout {
        width: parent.width

        TabBar {
            id: commandPathsBar
            Layout.fillWidth: true
            //Material.foreground: grey
            Material.accent: grey
            Material.background: Material.color(Material.BlueGrey, Material.Shade50)

            TabButton {
                id: commandsTabButton
                text: qsTr("Commands")

                /*background: Rectangle {
                    //implicitHeight: 40
                    opacity: enabled ? 1 : 0.3
                    color: cyan
            }*/

                contentItem: Text {
                    text: commandsTabButton.text
                    font: commandsTabButton.font
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    //opacity: enabled ? 1.0 : 0.3
                    color: (commandPathsBar.currentIndex == 0) ? cyan : grey
                }
            }

            TabButton {
                id: pathsTabButton
                text: qsTr("Paths")

                /*background: Rectangle {
                    //implicitHeight: 40
                    opacity: enabled ? 1 : 0.3
                    color: softorange
            }*/

                contentItem: Text {
                    text: pathsTabButton.text
                    font: pathsTabButton.font
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    //opacity: enabled ? 1.0 : 0.3
                    color: (commandPathsBar.currentIndex == 1) ? softorange : grey
                }
            }
        }

        StackLayout {
            currentIndex: commandPathsBar.currentIndex
            Layout.fillHeight: true
            Layout.fillWidth: true

            ColumnLayout {
                Layout.fillWidth: true

                id: commandsTab
                /*Label {
                text: "Commands"
                Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                Layout.topMargin: 10
                font.pointSize: 12
                font.weight: Font.DemiBold
                color: cyan
            }*/

                Button {
                    id: speedHeadButton
                    y: 42
                    text: "Speed/Heading"
                    Layout.fillWidth: true
                    Layout.fillHeight: false
                    highlighted: true
                    Material.background: pressed ? orange : mainColor
                    onClicked: {
                        cmdWrapper.sendSpeedHeadingCommand(sliderSpeed.value,
                                                           sliderHeading.value)
                    }
                }


                Button {
                    text: "Halt"
                    highlighted: true
                    Material.background: pressed ? orange : mainColor
                    Layout.fillHeight: false
                    Layout.fillWidth: true
                    onClicked: {
                        cmdWrapper.sendHaltCommand()
                    }
                }


                Button {
                    id: holdButton
                    text: "Hold Position"
                    Layout.fillWidth: true
                    antialiasing: false

                    highlighted: true
                    Material.background: pressed ? orange : mainColor

                    onClicked: {
                        cmdWrapper.sendHoldCommand(holdRadius.value)
                    }
                }

                Button {
                    id: moveToButton
                    text: "Move to marker"
                    Layout.fillWidth: true
                    antialiasing: false
                    Material.background: pressed ? orange : mainColor
                    enabled: Helper.coord_inside_polygon(map.marker_coords,
                                                         map.polysec_cur.path)
                             && (map.markerIcon.opacity > 0)
                    highlighted: true

                    onClicked: {
                        cmdWrapper.sendLatLongCommand(
                                    map.marker_coords,
                                    holdRadius.value)
                    }
                }
            }

            ColumnLayout {
                id: pathsTab
                Layout.fillWidth: true

                /*Label {
                id: pathLabel
                color: softorange
                text: "Paths"
                Layout.topMargin: 10
                antialiasing: false
                font.weight: Font.DemiBold
                Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                font.pointSize: 12
            }*/

                PathsCommands {
                    id: pathCommandsPane
                    Layout.bottomMargin: 10
                    Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                    Layout.fillWidth: true
                    Layout.fillHeight: false
                }
            }
        }
    }
}
