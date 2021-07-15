import QtQuick 2.6
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls.Material 2.1
import "."
import "../scripts/helper.js" as Helper

Pane{
    property alias pathCommandsPane: pathCommandsPane
    property alias speedHeadTimeout: speedHeadTimeout

    property var labelWidth: 100
    property var sliderWidth: 105
    property var unitsWidth: 40

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
                id: parametersTabButton
                text: qsTr("Parameters")

                contentItem: Text {
                    text: parametersTabButton.text
                    font: parametersTabButton.font
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    color: (commandPathsBar.currentIndex == 1) ? cyan : grey
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
                    id: holdButton
                    text: "Hold Position"
                    Layout.fillWidth: true
                    highlighted: true
                    Material.background: pressed ? orange : mainColor

                    onClicked: {
                        cmdWrapper.sendHoldCommand(holdRadius.value)
                    }
                }

                ToolSeparator {
                    orientation: Qt.Horizontal
                    Layout.fillWidth: true

                    contentItem: Rectangle {
                        implicitHeight: 1
                        color: "#c3c3c3"
                    }
                }

                PathsCommands {
                    id: pathCommandsPane
                    Layout.bottomMargin: 10
                    Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                    Layout.fillWidth: true
                    Layout.fillHeight: false
                }
            }

            ColumnLayout {
                id: parametersTab
                Layout.fillWidth: true

                RowLayout {
                    id: cruiseSpeedControl
                    Layout.alignment: Qt.AlignVCenter //| Qt.AlignHCenter

                    Label {
                        text: qsTr("Surge sat. ")
                        clip: false
                        leftPadding: 5
                        font.pointSize: 11
                        width: labelWidth
                        verticalAlignment: Text.AlignVCenter
                        horizontalAlignment: Text.AlignLeft
                    }

                    Slider {
                        id: sliderSpeed
                        objectName: "cruiseSpeed"
                        Layout.preferredWidth: sliderWidth + 20
                        from: 0.0
                        to: 5
                        stepSize: 0.1
                        value: 1.5
                        onPressedChanged: function() {
                            if (sliderSpeed.pressed === false)
                                cmdWrapper.setCruiseSpeedCommand(sliderSpeed.value)
                        }
                    }

                    Text {
                        text: sliderSpeed.value.toFixed(1) + " m/s"
                        verticalAlignment: Text.AlignVCenter
                        horizontalAlignment: Text.AlignLeft
                        //Layout.leftMargin: 10
                        Layout.preferredWidth: unitsWidth
                        font.pointSize: 11
                    }
                }

                RowLayout {
                    id: yawRateControl
                    Layout.alignment: Qt.AlignVCenter //| Qt.AlignHCenter

                    Label {
                        text: qsTr("YawRate sat.")
                        clip: false
                        leftPadding: 5
                        font.pointSize: 11
                        width: labelWidth
                        verticalAlignment: Text.AlignVCenter
                        horizontalAlignment: Text.AlignLeft
                    }

                    Slider {
                        id: sliderYawRate
                        objectName: "yawRateSat"
                        Layout.preferredWidth: sliderWidth - 5
                        from: 0.0
                        to: 2.0
                        stepSize: 0.1
                        value: 1
                        onPressedChanged: function() {
                            if (sliderYawRate.pressed === false)
                                cmdWrapper.setYawRateCommand(sliderYawRate.value)
                        }
                    }

                    Text {
                        text: sliderYawRate.value.toFixed(1) + " rad/s"
                        verticalAlignment: Text.AlignVCenter
                        horizontalAlignment: Text.AlignLeft
                        //Layout.leftMargin: 10
                        Layout.preferredWidth: unitsWidth
                        font.pointSize: 11
                    }
                }


                RowLayout {
                    id: cruiseHeadingControl
                    Layout.alignment: Qt.AlignVCenter //| Qt.AlignHCenter

                    Label {
                        text: qsTr("Heading")
                        leftPadding: 5
                        font.pointSize: 11
                        width: labelWidth
                        //Layout.fillWidth: true
                        horizontalAlignment: Text.AlignLeft
                        verticalAlignment: Text.AlignBottom
                    }

                    Slider {
                        id: sliderHeading
                        value: 0
                        Layout.preferredWidth: sliderWidth + 50
                        Layout.leftMargin: 0
                        stepSize: 1
                        from: 0
                        to: 359
                    }

                    Text {
                        width: 80
                        text: sliderHeading.value + " °"
                        font.pointSize: 11
                        horizontalAlignment: Text.AlignLeft
                        verticalAlignment: Text.AlignVCenter
                        Layout.preferredWidth: unitsWidth
                    }
                }

                RowLayout {
                    id: acceptRadControl
                    Layout.alignment: Qt.AlignVCenter //| Qt.AlignHCenter

                    Label {
                        text: qsTr("Accept. radius")
                        leftPadding: 5
                        font.pointSize: 11
                        width: labelWidth
                        horizontalAlignment: Text.AlignLeft
                        verticalAlignment: Text.AlignVCenter
                    }

                    Slider {
                        id: holdRadius
                        Layout.preferredWidth: sliderWidth
                        Layout.leftMargin: 0
                        value: 5
                        stepSize: 0.1
                        from: 0.5
                        to: 10
                    }

                    Text {
                        text: holdRadius.value.toFixed(1) + " m"
                        Layout.preferredWidth: unitsWidth
                        font.pointSize: 11
                        horizontalAlignment: Text.AlignLeft
                        verticalAlignment: Text.AlignVCenter
                    }
                }

                RowLayout {
                    id: commandDurationControl
                    //Layout.fillWidth: true
                    Layout.alignment: Qt.AlignVCenter //| Qt.AlignHCenter

                    Column{
                        Layout.preferredWidth: labelWidth - 10
                        Label {
                            text: qsTr("Cmd Timeout")
                            font.underline: false
                            leftPadding: 5
                            font.pointSize: 11
                            horizontalAlignment: Text.AlignLeft
                            verticalAlignment: Text.AlignBottom
                        }
                        Label {
                            text: qsTr("(0 = indefinite)")
                            font.underline: false
                            leftPadding: 5
                            font.pointSize: 8
                            horizontalAlignment: Text.AlignLeft
                            verticalAlignment: Text.AlignBottom
                        }
                    }

                    SpinBox {
                        id: speedHeadTimeout
                        objectName: "shTimeout"
                        editable: true
                        to: 1000
                        value: 200
                        Layout.preferredWidth: sliderWidth + 25
                        onValueChanged: function(){
                            settings.shTimeout = speedHeadTimeout.value
                        }
                    }

                    Text {
                        text: "s"
                        font.pointSize: 11
                        horizontalAlignment: Text.AlignLeft
                        verticalAlignment: Text.AlignVCenter
                        Layout.preferredWidth: unitsWidth
                    }
                }
            }
        }
    }
}
