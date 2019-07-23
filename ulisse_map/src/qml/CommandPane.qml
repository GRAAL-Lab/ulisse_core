import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.2

Pane {

    property var buttonSafety: buttonSafety

    property var trackComponent

    Component.onCompleted: {
        trackComponent = Qt.createComponent("ElementTrack.qml")
    }

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
                        if (cmdWrapper.sendLatLongCommand(
                                    map.marker_coords,
                                    parseFloat(moveToRadius.text))) {
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
            RowLayout {
                id: loadSavePath
                Button {
                    id: savePath
                    width: 255
                    height: 40
                    Layout.fillWidth: true
                    text: "Save Path"
                    transformOrigin: Item.Left
                    enabled: true

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
                id: buttonSafety
                text: qsTr("Define safety area")
                Layout.fillWidth: true

                function end(){
                    enabled = true
                    map.polysec_cur.end.disconnect(end)
                    map.click_handler = map.click_goto_handler
                    map.pos_changed_handler = function(){}
                    text = "Redefine safety area"
                    slidersLeft.check_safety_all()
                    cmdWrapper.sendBoundaries(JSON.stringify(map.polysec_cur.serialize()))
                }


                onClicked: function(){
                    map.polysec_cur.clear_path()
                    map.center = fbkUpdater.ulisse_pos
                    map.click_handler = map.polysec_cur.click_handler
                    map.pos_changed_handler = map.polysec_cur.pos_changed_handler
                    enabled = false
                    map.polysec_cur.end.connect(end)
                }
            }
        }
    }

    FileDialog {
        id: loadPathDialog
        title: "Please choose a file"
        folder: shortcuts.home
        nameFilters: ["Path Files (*.ulisse)"]

        onAccepted: {
            var path = loadPathDialog.fileUrl.toString()
            path = path.replace(/^(file:\/{2})/, "")
            console.log(path)
            loadPaths(path)
        }
    }

    FileDialog {
        id: savePathDialog
        title: "Saving path..."
        folder: shortcuts.home
        selectExisting: false
        nameFilters: ["Path Files (*.ulisse)"]

        onAccepted: {
            var path = savePathDialog.fileUrl.toString()
            path = path.replace(/^(file:\/{2})/, "") // remove prefixed "file://"
            path = decodeURIComponent(path) // unescape html codes like '%23' for '#'
            savePaths(path)
        }
    }

    function savePaths(filePath){
        var all_paths = {
            security_box: null, //TODO security box
            paths: []
        }

        for (var i=0; i<slidersLeft.columnTrack.children.length; i++){
            all_paths.paths.push(slidersLeft.columnTrack.children[i]._comp.serialize())
        }

        console.log("JSON to save "+JSON.stringify(all_paths))
        console.log("PATH to save "+filePath)

        cmdWrapper.savePathToFile(filePath, JSON.stringify(all_paths))
        //TODO show a toast
    }

    function loadPaths(filePath){
        var jsondata = cmdWrapper.loadPathFromFile(filePath)
        var data = JSON.parse(jsondata)

        var i,j,lat,lon,p

        slidersLeft.delete_all()

        for(i = 0; i < data.paths.length; i++){
            switch(data.paths[i].type){
            case "PolyPath":
                var cur_managed = map.createPoly()
                cur_managed.deserialize(data.paths[i])

                var v = trackComponent.createObject(slidersLeft.columnTrack)
                v._comp = cur_managed
                v.ntrack = slidersLeft.columnTrack.children.length
                v.selected.connect(function (path){
                    slidersLeft.update_selection(path)
                    pathRectPoly.manage(path)
                })

                cur_managed.draw_deferred()
                break

            case "PointPath":
                var cur_managed = map.createPath()
                cur_managed.deserialize(data.paths[i])

                var v = trackComponent.createObject(slidersLeft.columnTrack)
                v._comp = cur_managed
                v.ntrack = slidersLeft.columnTrack.children.length
                v.selected.connect(function (path){
                    slidersLeft.update_selection(path)
                    pathRectPoly.manage(path)
                })

                cur_managed.draw_deferred()
                break

            case "SecurityPoly":
                map.polysec_cur.clear_path()
                polysec_cur.deserialize(data.paths[i])
                polysec_cur.draw_deferred()
                break
            }
        }
    }
}
