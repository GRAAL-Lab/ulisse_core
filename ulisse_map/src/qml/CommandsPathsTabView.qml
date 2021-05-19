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
            Material.accent: grey
            Material.background: Material.color(Material.BlueGrey, Material.Shade50)

            TabButton {
                id: commandsTabButton
                text: qsTr("Commands")

                contentItem: Text {
                    text: commandsTabButton.text
                    font: commandsTabButton.font
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    color: (commandPathsBar.currentIndex == 0) ? cyan : grey
                }
            }

            TabButton {
                id: pathsTabButton
                text: qsTr("Paths")

                contentItem: Text {
                    text: pathsTabButton.text
                    font: pathsTabButton.font
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    color: (commandPathsBar.currentIndex == 1) ? softorange : grey
                }
            }
        }

        StackLayout {
            currentIndex: commandPathsBar.currentIndex
            Layout.fillHeight: true
            Layout.fillWidth: true

            ColumnLayout {
                id: commandsTab
                Layout.fillWidth: true

                Button {
                    id: moveToButton
                    text: "Move to marker"
                    Layout.fillWidth: true
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

                Button {
                    id: speedHeadButton
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
                    highlighted: true
                    Material.background: pressed ? orange : mainColor

                    onClicked: {
                        cmdWrapper.sendHoldCommand(holdRadius.value)
                    }
                }
            }

            ColumnLayout {
                id: pathsTab
                Layout.fillWidth: true

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
