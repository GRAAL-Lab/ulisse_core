import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.2

Pane {

    ColumnLayout {
        id: buttonsColumn
        anchors.verticalCenter: parent.verticalCenter
        anchors.horizontalCenter: parent.horizontalCenter
        width: parent.width
        spacing: 2

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
            Layout.minimumWidth: 150
            Layout.maximumWidth: 150
            Layout.preferredWidth: 150
            onClicked: {
                map.interruptPathIfActive()
                cmdWrapper.sendHaltCommand()
            }
        }

        RowLayout {
            id: speedHeadingRow
            Layout.fillWidth: true

            Button {
                id: speedHeadButton
                Layout.minimumWidth: 150
                Layout.maximumWidth: 150
                Layout.preferredWidth: 150
                text: "Speed-Heading"

                onClicked: {
                    if (speedText.text !== '' && headingText.text !== '') {
                        map.interruptPathIfActive()
                        cmdWrapper.sendSpeedHeadingCommand(speedText.text,
                                                           headingText.text)
                    } else {
                        speedHeadingDialog.open()
                    }
                }
            }

            Rectangle {
                id: speedHeadSpacer
                height: parent.height
                color: 'transparent'
                Layout.fillWidth: true
                Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
            }

            TextField {
                id: speedText
                Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                Layout.minimumWidth: 45
                Layout.maximumWidth: 45
                Layout.preferredWidth: 45

                font.pointSize: 10
                placeholderText: "S"
                selectByMouse: true

                ToolTip.text: qsTr("Speed (m/s)")
                ToolTip.delay: 500
                ToolTip.timeout: 5000
                ToolTip.visible: hovered

                validator: DoubleValidator {
                    bottom: -5.0
                    top: +5.0
                    decimals: 1
                    notation: DoubleValidator.StandardNotation
                }
            }

            TextField {
                id: headingText
                Layout.alignment: Qt.AlignRight | Qt.AlignVCenter
                Layout.minimumWidth: 45
                Layout.maximumWidth: 45
                Layout.preferredWidth: 45

                font.pointSize: 10
                placeholderText: "H°"
                selectByMouse: true

                ToolTip.text: qsTr("Heading (rad/s)")
                ToolTip.delay: 500
                ToolTip.timeout: 5000
                ToolTip.visible: hovered

                validator: IntValidator {
                    bottom: 0
                    top: 360
                }
            }
        }

        RowLayout {
            id: holdRow
            Layout.fillWidth: true

            Button {
                id: holdButton
                Layout.minimumWidth: 150
                Layout.maximumWidth: 150
                Layout.preferredWidth: 150
                text: "Hold Position"

                onClicked: {
                    if (holdRadius.text !== '') {
                        map.interruptPathIfActive()
                        cmdWrapper.sendHoldCommand(parseFloat(holdRadius.text))
                    } else {
                        acceptRadDialog.open()
                    }
                }
            }

            Rectangle {
                id: holdSpacer
                height: parent.height
                color: 'transparent'
                Layout.fillWidth: true
                Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
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
                text: "5.0"
                Layout.alignment: Qt.AlignRight | Qt.AlignVCenter

                ToolTip.text: qsTr("Acceptance Radius (meters)")
                ToolTip.delay: 500
                ToolTip.timeout: 5000
                ToolTip.visible: hovered

                validator: DoubleValidator {
                    bottom: 0.0
                    top: 101.0
                    decimals: 3
                    notation: DoubleValidator.StandardNotation
                }
            }
        }

        RowLayout {
            id: moveToRow
            Layout.fillWidth: true

            Button {
                id: moveToButton
                Layout.minimumWidth: 150
                Layout.maximumWidth: 150
                Layout.preferredWidth: 150
                text: "Move To Marker"
                enabled: map.markerIconOpacity > 0 ? true : false

                onClicked: {
                    if (moveToRadius.text !== '') {
                        map.interruptPathIfActive()
                        if (cmdWrapper.sendLatLongCommand(
                                    marker_coords,
                                    parseFloat(moveToRadius.text))) {
                            markerIcon.opacity = 0.2
                        }
                    } else {
                        acceptRadDialog.open()
                    }
                }
            }

            Rectangle {
                id: moveToSpacer
                height: parent.height
                color: 'transparent'
                Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                Layout.fillWidth: true
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
                Layout.alignment: Qt.AlignRight | Qt.AlignVCenter

                ToolTip.text: qsTr("Acceptance Radius (meters)")
                ToolTip.delay: 500
                ToolTip.timeout: 5000
                ToolTip.visible: hovered

                validator: DoubleValidator {
                    bottom: 0.0
                    top: 101.0
                    decimals: 3
                    notation: DoubleValidator.StandardNotation
                }
            }
        }

        Rectangle {
            id: divider
            property real margin: 8

            Layout.preferredHeight: 2
            Layout.preferredWidth: parent.width - margin * 2
            Layout.leftMargin: margin
            Layout.rightMargin: margin
            Layout.topMargin: margin
            Layout.bottomMargin: margin
            border.color: "lightgrey"
        }

        RowLayout {
            id: rectid
            PathRectPoly {
                id: pathRectPoly
            }

            RowLayout {
                id: loadSavePath
                Button {
                    id: savePath
                    width: 255
                    height: 40
                    Layout.fillWidth: true
                    text: "Save Path"
                    rotation: 1
                    transformOrigin: Item.Left
                    enabled: (mapView.pathCurrentState === pathState.empty)
                             | (mapView.pathCurrentState === pathState.creating) ? false : true

                    onClicked: {
                        savePathDialog.open()
                    }
                }

                Button {
                    id: loadPath
                    Layout.fillWidth: true
                    text: "Load Path"
                    enabled: (mapView.pathCurrentState === pathState.empty) ? true : false

                    onClicked: {
                        loadPathDialog.open()
                    }
                }
            }
        }

        RowLayout {
            id: additionalWpControls

            Button {
                id: goToPreviousWp
                text: "<"
                font.weight: Font.Bold
                font.pointSize: 16
                Layout.preferredWidth: 40
                enabled: mapView.pathCurrentState === pathState.active ? true : false

                ToolTip.text: qsTr("Go To Previous Waypoint")
                ToolTip.delay: 500
                ToolTip.timeout: 5000
                ToolTip.visible: hovered

                onClicked: {
                    cmdWrapper.goToPreviousWaypoint()
                }
            }

            Button {
                id: goToNextWp
                text: ">"
                font.weight: Font.Bold
                font.pointSize: 16
                Layout.preferredWidth: 40
                enabled: mapView.pathCurrentState === pathState.active ? true : false

                ToolTip.text: qsTr("Go To Next Waypoint")
                ToolTip.delay: 500
                ToolTip.timeout: 5000
                ToolTip.visible: hovered

                onClicked: {
                    cmdWrapper.goToNextWaypoint()
                }
            }

            CheckBox {
                objectName: "loopPath"
                id: loopPathCB
                text: "Loop over path"
                checked: false

                onCheckStateChanged: {
                    if (checked === true) {
                        wpCommands.loopPath = true
                    } else {
                        wpCommands.loopPath = false
                    }
                }
            }
        }






    }

    FileDialog {
        id: loadPathDialog
        title: "Please choose a file"
        folder: shortcuts.home
        nameFilters: ["Path Files (*.path)"]

        onAccepted: {
            map.deletePath()
            var path = loadPathDialog.fileUrl.toString()
            // remove prefixed "file://"
            path = path.replace(/^(file:\/{2})/, "")
            // unescape html codes like '%23' for '#'
            var cleanPath = decodeURIComponent(path)

            // console.log("Loaded file path: %1".arg(cleanPath))
            if (cmdWrapper.loadPathFromFile(cleanPath)) {
                mapView.pathCurrentState = pathState.empty
                wpCommands.wpButtonText = "Finalize..."
                wpCommands.wpButtonHighlighted = true
            }
        }
    }

    FileDialog {
        id: savePathDialog
        title: "Saving path..."
        folder: shortcuts.home
        selectExisting: false

        onAccepted: {
            var path = savePathDialog.fileUrl.toString()
            // remove prefixed "file://"
            path = path.replace(/^(file:\/{2})/, "")
            // unescape html codes like '%23' for '#'
            var cleanPath = decodeURIComponent(path)

            cmdWrapper.savePathToFile(cleanPath)
        }
    }
}
