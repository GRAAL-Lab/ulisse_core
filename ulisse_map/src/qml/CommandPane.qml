import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import QtQuick.Controls.Styles 1.4
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.2

Pane {
    property alias wpRad: wpCommands.wpRadius

    ColumnLayout {
        id: buttonsColumn
        anchors.verticalCenter: parent.verticalCenter
        anchors.horizontalCenter: parent.horizontalCenter
        width: parent.width
        spacing: 0

        Label {
            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
            font.pointSize: 12
            font.weight: Font.DemiBold
            bottomPadding: 10
            color: Material.color(Material.Green, Material.Shade700)
            text: "Commands"
        }


        Button {
            text: "Halt"
            onClicked: {
                wpCommands.interruptPath();
                cmdWrapper.sendHaltCommand()
            }
        }


        RowLayout {
            id: speedHeadingRow
            Layout.fillWidth: true

            Button {
                id: speedHeadButton
                text: "Speed-Heading"

                onClicked: {
                    if(speedText.text !== '' && headingText.text !== ''){
                        wpCommands.interruptPath();
                        cmdWrapper.sendSpeedHeadingCommand(speedText.text, headingText.text)
                    } else {
                        speedHeadingDialog.open();
                    }
                }

            }

            Rectangle {
                id: speedHeadSpacer
                width: buttonsColumn.width - speedHeadButton.width - speedText.width - headingText.width
                height: parent.height
                anchors.left: speedHeadButton.right
                color: 'transparent'
            }


            TextField {
                id: speedText
                Layout.maximumWidth: 40
                Layout.fillWidth: true
                anchors.left: speedHeadSpacer.right
                font.pointSize: 10
                placeholderText: "S"
                selectByMouse: true

                ToolTip.text: qsTr("Speed (m/s)")
                ToolTip.delay: 500
                ToolTip.timeout: 5000
                ToolTip.visible: hovered


                validator: DoubleValidator {
                    bottom: -5.0;
                    top: +5.0;
                    decimals: 1;
                    notation: DoubleValidator.StandardNotation
                }
            }

            TextField {
                id: headingText
                Layout.maximumWidth: 40
                Layout.fillWidth: true
                anchors.left: speedText.right
                font.pointSize: 10
                placeholderText: "H°"
                selectByMouse: true

                ToolTip.text: qsTr("Heading (m/s)")
                ToolTip.delay: 500
                ToolTip.timeout: 5000
                ToolTip.visible: hovered

                validator: IntValidator {
                    bottom: 0;
                    top: 360;

                }
            }
        }

        RowLayout {
            id: holdRow
            Layout.fillWidth: true

            Button {
                id: holdButton
                text: "Hold Position"

                onClicked: {
                    if(holdRadius.text !== ''){
                        wpCommands.interruptPath();
                        cmdWrapper.sendHoldCommand(parseFloat(holdRadius.text));
                    } else {
                        acceptRadDialog.open();
                    }
                }
            }

            Rectangle {
                id: holdSpacer
                width: buttonsColumn.width - holdButton.width - holdRadius.width
                height: parent.height
                anchors.left: holdButton.right
                color: 'transparent'
            }


            TextField {
                id: holdRadius
                objectName: "holdRadiusText"
                Layout.preferredWidth: 45
                Layout.minimumWidth: 45
                Layout.maximumWidth: 45
                font.pointSize: 10
                placeholderText: "Radius"
                selectByMouse: true
                anchors.left: holdSpacer.right
                text: "5.0"

                ToolTip.text: qsTr("Acceptance Radius (meters)")
                ToolTip.delay: 500
                ToolTip.timeout: 5000
                ToolTip.visible: hovered


                validator: DoubleValidator {
                    bottom: 0.0;
                    top: 50.0;
                    decimals: 1;
                    notation: DoubleValidator.StandardNotation
                }
            }
        }

        RowLayout {
            id: moveToRow
            Layout.fillWidth: true

            Button {
                id: moveToButton
                text: "Move To Marker"
                enabled: markerIcon.opacity > 0 ? true : false

                onClicked: {
                    if(moveToRadius.text !== ''){
                        wpCommands.interruptPath();
                        if(cmdWrapper.sendLatLongCommand(marker_coords, parseFloat(moveToRadius.text))){
                            markerIcon.opacity = 0.2
                        }
                    } else {
                        acceptRadDialog.open();
                    }
                }

            }

            Rectangle {
                id: moveToSpacer
                width: buttonsColumn.width - moveToButton.width - moveToRadius.width
                height: parent.height
                anchors.left: moveToButton.right
                color: 'transparent'
            }


            TextField {
                id: moveToRadius
                objectName: "moveToRadiusText"
                Layout.preferredWidth: 45
                Layout.minimumWidth: 45
                Layout.maximumWidth: 45
                font.pointSize: 10
                placeholderText: "Radius"
                selectByMouse: true
                text: "5.0"

                ToolTip.text: qsTr("Acceptance Radius (meters)")
                ToolTip.delay: 500
                ToolTip.timeout: 5000
                ToolTip.visible: hovered


                anchors.left: moveToSpacer.right
                validator: DoubleValidator {
                    bottom: 0.0;
                    top: 50.0;
                    decimals: 1;
                    notation: DoubleValidator.StandardNotation
                }
            }
        }

        Rectangle {
            property real margin: 8

            Layout.preferredHeight: 2
            Layout.preferredWidth: parent.width - margin*2
            Layout.leftMargin: margin
            Layout.rightMargin: margin
            Layout.topMargin: 6
            Layout.bottomMargin: 6
            border.color: "lightgrey"
        }

        WaypointControls {
            id: wpCommands
            Layout.fillWidth: true
        }


        CheckBox {
            objectName: "loopPath"
            id: loopPathCB
            text: "Loop over path"
            //anchors.left: followMeCheckbox.right
            //Material.accent: mainColor
            checked: false

            onCheckStateChanged: {
                if (checked === true){
                    wpCommands.loopPath = true;
                } else {
                    wpCommands.loopPath = false;
                }
            }
        }

        RowLayout {
            id: additionalWpControls
            Button {
                id: savePath
                text: "Save Path"
                enabled: (mapView.pathCurrentState === pathState.empty) |  (mapView.pathCurrentState === pathState.creating) ? false : true

                onClicked: {
                    savePathDialog.open()
                }
            }

            Button {
                id: loadPath
                text: "Load Path"
                enabled: (mapView.pathCurrentState === pathState.empty) ? true : false

                onClicked: {
                    loadPathDialog.open()
                }
            }

        }

        FileDialog {
            id: loadPathDialog
            title: "Please choose a file"
            folder: shortcuts.home
            nameFilters: ["Path Files (*.path)"]

            onAccepted: {
                if(cmdWrapper.loadPathFromFile(loadPathDialog.fileUrls)){
                    mapView.pathCurrentState = pathState.empty;
                    wpCommands.wpButtonText = "Confirm"
                    wpCommands.wpButtonHighlighted = true;
                    console.log(("Load Path: pathLength = %1").arg(waypointPath.pathLength()))
                }
            }
        }

    }
}
