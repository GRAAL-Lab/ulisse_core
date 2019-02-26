import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import QtQuick.Controls.Styles 1.4
import QtGraphicalEffects 1.0

Pane {
    property alias wpRad: waypointRadius.text

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
            onClicked: cmdWrapper.sendHaltCommand()
        }

        RowLayout {
            id: speedHeadingRow
            Layout.fillWidth: true

            Button {
                id: speedHeadButton
                text: "Speed-Heading"

                onClicked: {
                    if(speedText.text !== '' && headingText.text !== ''){
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

                onClicked: {
                    if(moveToRadius.text !== ''){
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

                anchors.left: moveToSpacer.right
                validator: DoubleValidator {
                    bottom: 0.0;
                    top: 50.0;
                    decimals: 1;
                    notation: DoubleValidator.StandardNotation
                }
            }
        }

        RowLayout {
            id: waypointsRow
            Layout.fillWidth: true

            Button {
                id: waypointsButton
                Material.accent: secondaryAccentColor
                text: "Create Path"
                Layout.preferredWidth: 110

                onClicked: {
                    if(waypointRadius.text !== ''){
                        if (mapView.pathCurrentState === pathState.empty){
                            waypointsButton.text = "Send Path"
                            waypointsButton.highlighted = true;
                            mapView.pathCurrentState = pathState.creating;
                        }
                        else if (mapView.pathCurrentState === pathState.creating) {
                            waypointsButton.text = "Pause Path"
                            waypointsButton.Material.accent = mainAccentColor;
                            greenFlag.coordinate = waypointPath.path[waypointPath.pathLength() - 1];
                            mapView.pathCurrentState = pathState.active;
                            cmdWrapper.startPath();
                        }
                        else if (mapView.pathCurrentState === pathState.active) {
                            waypointsButton.text = "Resume Path"
                            mapView.pathCurrentState = pathState.stopped;
                            cmdWrapper.stopPath();
                        }
                        else if (mapView.pathCurrentState === pathState.stopped) {
                            waypointsButton.text = "Pause Path"
                            mapView.pathCurrentState = pathState.active;
                            cmdWrapper.resumePath();
                        }

                    } else {
                        acceptRadDialog.open();
                    }
                }
            }

            Button {
                id: wpRestartButton
                Material.accent: secondaryAccentColor
                Layout.preferredWidth: 28
                enabled: (mapView.pathCurrentState === pathState.active) || (mapView.pathCurrentState === pathState.stopped)  ? true : false

                ToolTip.text: qsTr("Restart path from beginning")
                ToolTip.delay: 500
                ToolTip.timeout: 5000
                ToolTip.visible: hovered

                Image {
                    id: restartIco
                    anchors.fill: parent
                    source: wpRestartButton.enabled ? "qrc:/images/restart-256.png" : "qrc:/images/restart-256_grey.png"
                    mipmap: true
                    fillMode: Image.PreserveAspectFit
                    horizontalAlignment: Image.AlignHCenter
                    verticalAlignment: Image.AlignVCenter
                }

                onClicked: {
                    cmdWrapper.startPath();
                }
            }

            Button {
                id: wpDeleteButton
                Material.accent: Material.Red
                highlighted: true
                Layout.preferredWidth: 28
                text: "X"
                font.weight: Font.Bold
                font.pointSize: 12
                enabled: mapView.pathCurrentState === pathState.empty ? false : true

                ToolTip.text: qsTr("Delete the current path and stop")
                ToolTip.delay: 500
                ToolTip.timeout: 5000
                ToolTip.visible: hovered

                onClicked: {
                    while (waypointPath.pathLength() > 0){
                        map.removeMapItem(mapCircles[waypointPath.pathLength() - 1]);
                        mapCircles[waypointPath.pathLength() - 1].destroy();
                        waypointPath.removeCoordinate(waypointPath.pathLength() - 1);
                    }

                    console.log(("Destroyed Path: pathLength = %1").arg(waypointPath.pathLength()))
                    waypointsButton.text = "Create Path";
                    waypointsButton.highlighted = false;
                    waypointsButton.Material.accent = secondaryAccentColor;
                    waypointPath.opacity = 0.0;
                    mapView.pathCurrentState = pathState.empty;

                    cmdWrapper.cancelPath();
                }
            }

            Rectangle {
                id: waypointsSpacer
                width: buttonsColumn.width - waypointsButton.width - waypointRadius.width - wpDeleteButton.width - wpRestartButton.width - 10
                height: parent.height
                anchors.left: wpDeleteButton.right
                color: 'transparent'
            }


            TextField {
                id: waypointRadius
                objectName: "waypointRadius"
                Layout.preferredWidth: 45
                Layout.minimumWidth: 45
                Layout.maximumWidth: 45
                font.pointSize: 10
                placeholderText: "Radius"
                selectByMouse: true
                enabled: mapView.pathCurrentState === pathState.empty |  mapView.pathCurrentState === pathState.creating ? true : false

                anchors.left: waypointsSpacer.right
                validator: DoubleValidator {
                    bottom: 0.0;
                    top: 50.0;
                    decimals: 1;
                    notation: DoubleValidator.StandardNotation
                }
            }
        }
    }
}
