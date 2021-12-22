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

    property var safetyPoly_bkp: []
    property var buttonSafety: buttonBoundBoxDefine
    property var pathButtonComponent
    property alias speedHeadTimeout: commandParamsStackContainer.speedHeadTimeout
    property alias pathCmdPane: commandParamsStackContainer.pathCommandsPane

    Component.onCompleted: {
        pathButtonComponent = Qt.createComponent("PathButton.qml")
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
                        cmdWrapper.sendBoundaries(JSON.stringify(map.safety_polygon.serialize()))
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

                    onClicked: {
                        /*if(map.safety_polygon == undefined){
                            map.createSafetyPolygon()
                        }*/

                        safetyPoly_bkp = map.safety_polygon.path
                        map.safety_polygon.clear_path()
                        map.center = fbkUpdater.ulisse_pos
                        map.clickHandler = map.safety_polygon.clickHandler
                        map.posChangedHandler = map.safety_polygon.posChangedHandler
                        enabled = false

                        map.mapTextOverlay.text = "Press ESC to cancel"
                        map.mapTextOverlay.visible = true

                        window.sig_escape.connect(reset_safetypoly)
                        map.safety_polygon.end.connect(end)
                    }

                    function end() {
                        enabled = true
                        map.mapTextOverlay.visible = false
                        map.safety_polygon.end.disconnect(end)
                        window.sig_escape.disconnect(reset_safetypoly)
                        map.clickHandler = map.click_goto_handler
                        map.posChangedHandler = function () {}
                        text = "Redefine"
                        buttonBoundBoxResend.enabled = true
                        commandParamsStackContainer.pathCommandsPane.check_safety_all()
                        cmdWrapper.sendBoundaries(JSON.stringify(map.safety_polygon.serialize()))
                        //settings.savedBoundary = map.safety_polygon
                    }

                    function reset_safetypoly(){
                        enabled = true
                        map.mapTextOverlay.visible = false
                        map.safety_polygon.end.disconnect(end)
                        window.sig_escape.disconnect(reset_safetypoly)
                        map.safety_polygon.path = safetyPoly_bkp
                        map.clickHandler = map.click_goto_handler
                        map.posChangedHandler = function () {}
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
                    onClicked: cmdWrapper.sendBoundaries(JSON.stringify(map.safety_polygon.serialize()))
                }
            }
        }

        CommandsTabView {
            // COMMAND BUTTONS ARE INSIDE THIS QML

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
            text: "(Shortcut: Return)"
            verticalAlignment: Text.AlignVCenter
            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignHCenter
        }
    }


}
