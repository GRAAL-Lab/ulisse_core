import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import QtQuick.Controls.Styles 1.4

Pane {
    property alias wpRad: waypointsText.text

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
                    if(holdRadiusText.text !== ''){
                        cmdWrapper.sendHoldCommand()
                    } else {
                        acceptRadDialog.open();
                    }
                }
            }

            Rectangle {
                id: holdSpacer
                width: buttonsColumn.width - holdButton.width - holdRadiusText.width
                height: parent.height
                anchors.left: holdButton.right
                color: 'transparent'
            }


            TextField {
                id: holdRadiusText
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
                    if(moveToRadiusText.text !== ''){
                        if(cmdWrapper.sendLatLongCommand(marker_coords)){
                            markerIcon.opacity = 0.2
                        }
                    } else {
                        acceptRadDialog.open();
                    }
                }

            }

            Rectangle {
                id: moveToSpacer
                width: buttonsColumn.width - moveToButton.width - moveToRadiusText.width
                height: parent.height
                anchors.left: moveToButton.right
                color: 'transparent'
            }


            TextField {
                id: moveToRadiusText
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

            //property bool creatingPath: false

            Button {
                id: waypointsButton
                Material.accent: secondaryAccentColor
                text: "Create Path"

                onClicked: {
                    if(waypointsText.text !== ''){
                        //console.log(("Before: createPathState = %1").arg(mapView.createPathState))

                        if (mapView.createPathState === pathState.empty){
                            // TODO: Set Up waypoint path (MouseArea + MapPolyline)
                            waypointsButton.text = "Send Path"
                            waypointsButton.highlighted = true;
                            mapView.createPathState = pathState.creating;
                        }
                        else if (mapView.createPathState === pathState.creating) {
                            // Send Waypoints (Create a Function in commandWrapper)
                            waypointsButton.text = "Cancel Path"
                            waypointsButton.Material.accent = mainAccentColor;

                            mapView.createPathState = pathState.active;
                        }
                        else if (mapView.createPathState === pathState.active) {

                            //var pathSize = waypointPath.pathLength();
                            //console.log(("Begin: pathSize = %1").arg(pathSize))
                            while (waypointPath.pathLength() > 0){
                                //waypointPath.opacity = 0.0;
                                map.removeMapItem(mapCircles[waypointPath.pathLength() - 1]);
                                mapCircles[waypointPath.pathLength() - 1].destroy();
                                waypointPath.removeCoordinate(waypointPath.pathLength() - 1);
                            }

                            console.log(("Destroyed Path: pathLength = %1").arg(waypointPath.pathLength()))
                            waypointsButton.text = "Create Path";
                            waypointsButton.highlighted = false;
                            waypointsButton.Material.accent = secondaryAccentColor;

                            mapView.createPathState = pathState.empty;
                        }


                        toast.show("Not yet implemented", 2000)
                    } else {
                        acceptRadDialog.open();
                    }
                }
            }

            Rectangle {
                id: waypointsSpacer
                width: buttonsColumn.width - waypointsButton.width - waypointsText.width
                height: parent.height
                anchors.left: waypointsButton.right
                color: 'transparent'
            }


            TextField {
                id: waypointsText
                objectName: "waypointsText"
                Layout.preferredWidth: 45
                Layout.minimumWidth: 45
                Layout.maximumWidth: 45
                font.pointSize: 10
                placeholderText: "Radius"
                selectByMouse: true

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
