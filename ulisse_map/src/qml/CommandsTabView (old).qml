import QtQuick 2.6
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls.Material 2.1
import QtPositioning 5.6
import "."
import "../scripts/helper.js" as Helper


Rectangle {

    Layout.alignment: Qt.AlignCenter
    //width: commandsColumnLayout.width + 10
    //height: commandsColumnLayout.height + 10
    color: dimmedwhite
    border.color: lightgrey
    border.width: 1

    property alias pathCommandsPane: pathCommandsPane
    property alias speedHeadTimeout: speedHeadTimeout
    property alias commandsColumnLayout: commandsColumnLayout

    property var sliderHeading: sliderHeading

    property real labelWidth: 100
    property real sliderWidth: 105
    property real unitsWidth: 40

    ColumnLayout {
        id: commandsColumnLayout

        //anchors.horizontalCenter: parent.horizontalCenter
        //anchors.top: parent.top
        anchors.fill: parent
        anchors.margins: 1

        Label {
            font.pointSize: 13
            font.weight: Font.DemiBold
            color: mainColor
            text: "Commands"
            verticalAlignment: Text.AlignVCenter
            horizontalAlignment: Text.AlignHCenter
            Layout.topMargin: 10
            Layout.bottomMargin: 5
            Layout.fillWidth: true
            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
        }

        TabBar {
            id: commandPathsBar
            Layout.fillWidth: true
            Material.accent: mainColor
            //Material.background: Material.color(Material.BlueGrey, Material.Shade50)

            TabButton {
                id: commandsTabButton
                text: qsTr("Geographic")

                contentItem: Text {
                    text: commandsTabButton.text
                    font: commandsTabButton.font
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    color: (commandPathsBar.currentIndex === 0) ? mainColor : grey
                }

                background: Rectangle {
                    color: (commandPathsBar.currentIndex === 0) ? lighterbluegrey : lightbluegrey
                }
            }

            TabButton {
                id: parametersTabButton
                text: qsTr("Parametric")

                contentItem: Text {
                    text: parametersTabButton.text
                    font: parametersTabButton.font
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    color: (commandPathsBar.currentIndex === 1) ? mainColor : grey
                }

                background: Rectangle {
                    color: (commandPathsBar.currentIndex === 1) ? lighterbluegrey : lightbluegrey
                }
            }
        }

        StackLayout {
            currentIndex: commandPathsBar.currentIndex
            Layout.fillHeight: true
            Layout.fillWidth: true
            Layout.margins: 10

            ColumnLayout {
                id: commandsTab
                Layout.fillWidth: true

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


                RowLayout {
                    Button {
                        id: moveToButton
                        text: "Move to Coordinate"
                        Layout.fillWidth: true
                        Material.background: pressed ? orange : mainColor
                        highlighted: true

                        onClicked: {
                            var coords_tokens = moveToCoordinate.latLongText.split(/[ ,]+/);
                            var coord = QtPositioning.coordinate(coords_tokens[0], coords_tokens[1])

                            if (coord.isValid) {
                            toast.show("Moving to coordinate:\n"
                                       + coord.latitude.toFixed(7) + ", " + coord.longitude.toFixed(7), 6000)
                            cmdWrapper.sendLatLongCommand(
                                        coord,
                                        holdRadius.value)
                            } else {
                                toast.show("Invalid coordinate.")
                            }
                        }


                    }

                    Button {
                        id: moveToMarkerButton
                        text: "\ue820" // icon-folder
                        font.family: "fontello"
                        font.pointSize: 12
                        padding: 5
                        Layout.minimumWidth: 18
                        Material.background: pressed ? orange : mainColor
                        enabled: Helper.coord_inside_polygon(map.marker_coords,
                                                             map.safety_polygon.path)
                                 && (map.markerIcon.opacity > 0)
                        highlighted: true
                        ToolTip.delay: 1000
                        ToolTip.timeout: 5000
                        ToolTip.visible: hovered
                        ToolTip.text: qsTr("Move to marker")

                        onClicked: {
                            toast.show("Moving to coordinate:\n"
                                       + map.marker_coords.latitude.toFixed(7) + ", " + map.marker_coords.longitude.toFixed(7), 6000)
                            cmdWrapper.sendLatLongCommand(
                                        map.marker_coords,
                                        holdRadius.value)
                        }
                    }
                }

                RowLayout {
                    id: moveToCoordinate

                    property alias latLongText: latLongText.text
                    spacing: 10
                    //enabled: settings.mapPluginType === "esri" ? true : false

                    Label {
                        font.pointSize: 11
                        text: "Lat, Long:"
                    }

                    TextField {
                        property bool changed: false

                        id: latLongText
                        Layout.preferredWidth: 15
                        Layout.fillWidth: true
                        font.pointSize: 11
                        text: ""
                        placeholderText: "Latitude, Longitude"
                        selectByMouse: true

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

                Rectangle {
                    Layout.fillWidth: true
                    implicitHeight: 1
                    color: "#c3c3c3"
                }

                //ToolSeparator {
                //    orientation: Qt.Horizontal
                //    Layout.fillWidth: true

                //    contentItem: Rectangle {
                //        implicitHeight: 1
                //        color: "#c3c3c3"
                //    }
                //}

                PathsCommands {
                    id: pathCommandsPane
                    //Layout.bottomMargin: 10
                    Layout.alignment: Qt.AlignHCenter | Qt.AlignTop
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                }
            }

            ColumnLayout {
                id: parametersTab
                Layout.fillWidth: true


                Button {
                    id: speedHeadButton
                    text: "Surge/Heading"
                    Layout.fillWidth: true
                    Layout.fillHeight: false
                    highlighted: true
                    Material.background: pressed ? orange : mainColor
                    onClicked: {
                        cmdWrapper.sendSurgeHeadingCommand(sliderSurge.value,
                                                           sliderHeading.value)
                    }
                }

                Button {
                    id: surgeYawRateButton
                    text: "Surge/YawRate"
                    //enabled: false
                    Layout.fillWidth: true
                    Layout.fillHeight: false
                    highlighted: true
                    Material.background: pressed ? orange : mainColor
                    onClicked: {
                        cmdWrapper.sendSurgeYawRateCommand(sliderSurge.value,
                                                           sliderYawRate.value)
                    }
                }

                RowLayout {
                    id: surgeControl
                    Layout.alignment: Qt.AlignVCenter //| Qt.AlignHCenter

                    Label {
                        text: qsTr("Surge Ref. ")
                        clip: false
                        leftPadding: 5
                        font.pointSize: 11
                        width: labelWidth
                        verticalAlignment: Text.AlignVCenter
                        horizontalAlignment: Text.AlignLeft
                    }

                    Slider {
                        id: sliderSurge
                        objectName: "cruiseSpeed"
                        Layout.preferredWidth: sliderWidth + 20
                        from: 0.0
                        to: 5
                        stepSize: 0.1
                        value: 1.5
                        /*onPressedChanged: function() {
                            if (sliderSurge.pressed === false)
                                cmdWrapper.setCruiseSpeedCommand(sliderSurge.value)
                        }*/
                    }

                    Text {
                        text: sliderSurge.value.toFixed(1) + " m/s"
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
                        text: qsTr("YawRate Ref.")
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
                        to: 1.0
                        stepSize: 0.05
                        value: 0.5
                        /*onPressedChanged: function() {
                            if (sliderYawRate.pressed === false)
                                cmdWrapper.setYawRateCommand(sliderYawRate.value)
                        }*/
                    }

                    Text {
                        text: sliderYawRate.value.toFixed(2) + " rad/s"
                        verticalAlignment: Text.AlignVCenter
                        horizontalAlignment: Text.AlignLeft
                        //Layout.leftMargin: 10
                        Layout.preferredWidth: unitsWidth
                        font.pointSize: 11
                    }
                }


                RowLayout {
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
