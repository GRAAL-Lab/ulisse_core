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

    font.pointSize: 8
    Layout.fillWidth: true
    Layout.fillHeight: true
    Component.onCompleted: {
        trackComponent = Qt.createComponent("ElementTrack.qml")
    }

    ColumnLayout {
        id: buttonsColumn
        anchors.fill: parent
        spacing: 0

        Label {
            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
            font.pointSize: 12
            font.weight: Font.DemiBold
            color: Material.color(Material.Green, Material.Shade700)
            text: "Commands"
            antialiasing: false
        }

        Button {
            width: parent.width
            text: "Halt"
            antialiasing: false
            Layout.fillWidth: true
            Layout.fillHeight: true
            font.capitalization: Font.AllUppercase
            onClicked: {
                cmdWrapper.sendHaltCommand()
            }
        }

        Rectangle {
            id: rectangle3
            height: 5
            color: "#cdcdcd"
            Layout.fillWidth: true
        }

        RowLayout {
            id: speedHeadingRow
            antialiasing: true
            Layout.fillWidth: true
            Layout.fillHeight: true

            TextField {
                id: speedText
                antialiasing: true
                Layout.fillHeight: true
                Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                Layout.fillWidth: true
                placeholderText: "S"
                selectByMouse: true
                horizontalAlignment: TextInput.AlignHCenter

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
                antialiasing: true
                Layout.fillHeight: true
                Layout.alignment: Qt.AlignRight | Qt.AlignVCenter
                Layout.fillWidth: true

                placeholderText: "H°"
                selectByMouse: true
                horizontalAlignment: TextInput.AlignHCenter

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

        Button {
            id: speedHeadButton
            text: "Speed-Heading"
            antialiasing: false
            Layout.fillWidth: true
            Layout.fillHeight: true
            font.capitalization: Font.AllUppercase

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
            id: rectangle1
            height: 5
            color: "#cdcdcd"
            Layout.fillWidth: true
        }

        TextField {
            id: holdRadius
            objectName: "holdRadiusText"
            Layout.fillWidth: true
            Layout.fillHeight: false

            placeholderText: "Radius"
            selectByMouse: true
            text: "5.0"
            antialiasing: true
            Layout.alignment: Qt.AlignRight | Qt.AlignVCenter
            horizontalAlignment: TextInput.AlignHCenter

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

        Button {
            id: holdButton
            text: "Hold Position"
            Layout.fillWidth: true
            Layout.fillHeight: true
            font.capitalization: Font.AllUppercase
            padding: 5
            antialiasing: false

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
            id: rectangle
            height: 5
            color: "#cdcdcd"
            Layout.fillHeight: false
            Layout.fillWidth: true
        }

        TextField {
                id: moveToRadius
                objectName: "moveToRadiusText"
                Layout.fillWidth: true
                Layout.fillHeight: false
                placeholderText: "Radius"
                selectByMouse: true
                text: "5.0"
                padding: 0
                antialiasing: true
                Layout.alignment: Qt.AlignRight | Qt.AlignVCenter
                horizontalAlignment: TextInput.AlignHCenter

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

        Button {
            id: moveToButton
            Layout.fillWidth: true
            Layout.fillHeight: true
            text: "Move To Marker"
            enabled: true
            antialiasing: false

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
            id: rectangle2
            height: 5
            color: "#cdcdcd"
            Layout.fillWidth: true
        }

        RowLayout {
            id: rectid
            RowLayout {
                id: loadSavePath
                Button {
                    id: savePath
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    text: "Save Path"
                    antialiasing: false
                    transformOrigin: Item.Left
                    enabled: true

                    onClicked: {
                        savePathDialog.open()
                    }
                }

                Button {
                    id: loadPath
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    text: "Load Path"
                    focusPolicy: Qt.WheelFocus
                    transformOrigin: Item.Right
                    antialiasing: false
                    enabled: (mapView.pathCurrentState === pathState.empty) ? true : false

                    onClicked: {
                        loadPathDialog.open()
                    }
                }
            }
        }

        RowLayout {
            id: additionalWpControls
            spacing: 0
            Layout.fillHeight: true
            Layout.fillWidth: true

            Button {
                id: buttonSafety
                text: qsTr("Define safety area")
                padding: 5
                antialiasing: false
                Layout.fillWidth: true
                Layout.fillHeight: true

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
        if(slidersLeft.columnTrack.children.length === 0)
        {
            toast.show("There is nothing to save!")
            return
        }

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
