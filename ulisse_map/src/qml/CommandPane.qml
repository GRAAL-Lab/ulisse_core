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
    property alias speedHeadTimeout: commandParamsStackContainer.speedHeadTimeout
    property alias buttonSafety1: buttonBoundBoxResend
    property alias sweepPathCmdPane: commandParamsStackContainer.pathCommandsPane

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
                font.pointSize: 11
                //Layout.topMargin: 20
                font.weight: Font.DemiBold
            }

            RowLayout {
                id: boundingBoxButtons
                Layout.fillWidth: true

                /*Button {
                    id: buttonBoundBoxGet
                    text: qsTr("Get")
                    font.pointSize: 9
                    padding: 5
                    antialiasing: false
                    Layout.fillWidth: true
                    highlighted: true
                    Material.background: green
                    //Layout.fillHeight: false
                    onClicked: {
                        cmdWrapper.sendBoundaries(JSON.stringify(map.polysec_cur.serialize()))
                    }
                }*/

                Button {
                    id: buttonBoundBoxDefine
                    text: qsTr("Define")
                    font.pointSize: 9
                    padding: 5
                    antialiasing: false
                    Layout.fillWidth: true
                    //Layout.fillHeight: false
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
                        commandParamsStackContainer.pathCommandsPane.check_safety_all()
                        cmdWrapper.sendBoundaries(JSON.stringify(map.polysec_cur.serialize()))
                        settings.savedBoundary = JSON.stringify(map.polysec_cur.serialize())
                    }

                    function reset_polysec(){
                        enabled = true
                        map.polysec_cur.end.disconnect(end)
                        window.sig_escape.disconnect(reset_polysec)
                        map.polysec_cur.path = polysec_bkp
                        map.click_handler = map.click_goto_handler
                        map.pos_changed_handler = function () {}
                    }

                    onClicked: {
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
                    font.pointSize: 9
                    padding: 5
                    antialiasing: false
                    Layout.fillWidth: true
                    highlighted: true
                    //Layout.fillHeight: false
                    onClicked: cmdWrapper.sendBoundaries(JSON.stringify(map.polysec_cur.serialize()))
                }
            }
        }




        CommandsParamsTabView {
            // This Item is needed to add margins to the StackLayout and make it correctly resize
            id: commandParamsStackContainer
            Layout.topMargin: 10
            Layout.bottomMargin: 20
            Layout.fillWidth: true
            Layout.fillHeight: true
            Material.background: Material.color(Material.BlueGrey, Material.Shade50)
        }

        Button {
            text: "Halt"
            highlighted: true
            Material.background: orange//pressed ? orange : mainColor
            Layout.fillHeight: false
            Layout.fillWidth: true
            onClicked: {
                cmdWrapper.sendHaltCommand()
            }
        }

        Text {
            id: haltText
            font.pointSize: 8
            color: 'grey'
            text: "(Shortcut: Spacebar)"
            verticalAlignment: Text.AlignVCenter
            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignHCenter
        }
    }


}
