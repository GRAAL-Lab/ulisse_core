import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import QtGraphicalEffects 1.0

import "."
import "../scripts/helper.js" as Helper

Pane {

    property var polysec_bkp: []
    property var buttonSafety: buttonBoundBoxDefine
    property var trackComponent
    property alias speedHeadTimeout: speedHeadTimeout
    property alias buttonSafety1: buttonBoundBoxResend
    property alias sweepPathCmdPane: commandPathsStackContainer.pathCommandsPane

    property var labelWidth: 100
    property var sliderWidth: 130
    property var unitsWidth: 40

    Component.onCompleted: {
        trackComponent = Qt.createComponent("PathButton.qml")
    }

    ColumnLayout {
        id: controlsColumn
        anchors.fill: parent
        spacing: 1

        RowLayout {
            id: boundingBoxControls
            Layout.fillWidth: true

            Label {
                color: green
                text: "Safety area"
                antialiasing: false
                Layout.alignment: Qt.AlignVCenter //| Qt.AlignVCenter
                //Layout.bottomMargin: 20
                font.pointSize: 12
                //Layout.topMargin: 20
                font.weight: Font.DemiBold
            }

            RowLayout {
                id: boundingBoxButtons
                Layout.fillWidth: true
                Button {
                    id: buttonBoundBoxDefine
                    text: qsTr("Define")

                    padding: 5
                    antialiasing: false
                    Layout.fillWidth: true
                    Layout.fillHeight: false

                    highlighted: true
                    Material.background: green

                    function end() {
                        enabled = true
                        map.polysec_cur.end.disconnect(end)
                        window.sig_escape.disconnect(reset_polysec)
                        map.click_handler = map.click_goto_handler
                        map.pos_changed_handler = function () {}
                        text = "Redefine"
                        buttonBoundBoxResend.enabled = true
                        commandPathsStackContainer.pathCommandsPane.check_safety_all()
                        cmdWrapper.sendBoundaries(JSON.stringify(map.polysec_cur.serialize()))
                    }

                    function reset_polysec(){
                        enabled = true
                        map.polysec_cur.end.disconnect(end)
                        window.sig_escape.disconnect(reset_polysec)
                        map.polysec_cur.path = polysec_bkp
                        map.click_handler = map.click_goto_handler
                        map.pos_changed_handler = function () {}
                    }

                    onClicked: function () {
                        polysec_bkp = map.polysec_cur.path
                        map.polysec_cur.clear_path()
                        map.center = fbkUpdater.ulisse_pos
                        map.click_handler = map.polysec_cur.click_handler
                        map.pos_changed_handler = map.polysec_cur.pos_changed_handler
                        enabled = false
                        window.sig_escape.connect(reset_polysec)
                        map.polysec_cur.end.connect(end)
                    }
                }

                Button {
                    id: buttonBoundBoxResend
                    enabled: false
                    text: qsTr("Resend")
                    padding: 5
                    antialiasing: false
                    Layout.fillWidth: true
                    highlighted: true
                    Layout.fillHeight: false
                    onClicked: cmdWrapper.sendBoundaries(JSON.stringify(map.polysec_cur.serialize()))
                }
            }
        }

        ColumnLayout {
            Layout.topMargin: 5
            Layout.bottomMargin: 10
            spacing: 0
            Label {
                text: "Parameters"
                color: grey
                //Layout.bottomMargin: 10
                Layout.topMargin: 10
                font.weight: Font.DemiBold
                Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                font.pointSize: 12
            }

            RowLayout {
                id: cruiseSpeedControl
                Layout.alignment: Qt.AlignVCenter //| Qt.AlignHCenter

                Label {
                    text: qsTr("Cruise speed  ")
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
                    Layout.preferredWidth: sliderWidth
                    to: 5
                    from: 0.0
                    stepSize: 0.1
                    value: 1
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
                    stepSize: 0.1
                    from: 0
                    to: 359.9
                }

                Text {
                    width: 80
                    text: sliderHeading.value.toFixed(1) + " °"
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
                        text: qsTr("Duration")
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
        //}


        CommandsPathsTabView {
            // This Item is needed to add margins to the StackLayout and make it correctly resize
            id: commandPathsStackContainer
            Layout.fillWidth: true
            Layout.fillHeight: true
            //visible: false
        }
    }


}
