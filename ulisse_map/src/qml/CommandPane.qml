import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.2

import "../scripts/helper.js" as Helper


Pane {

    property var buttonSafety: buttonSafety

    property var trackComponent

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

            highlighted: true
            Material.background: pressed? mainAccentColor : Material.color(Material.Blue, Material.Shade700)

            font.capitalization: Font.AllUppercase

            Layout.fillHeight: false

            onClicked: {
                cmdWrapper.sendHaltCommand()
            }
        }


        Rectangle {
            clip:true
            id: rectangle0
            width: 200
            height:90
            color: "#ffffff"
            Layout.fillHeight: false
            Layout.fillWidth: true

            RowLayout {
                id: speedHeadingRow
                anchors.top: parent.top
                anchors.right: parent.right
                anchors.rightMargin: 0
                anchors.left: parent.left
                anchors.leftMargin: 0
                anchors.topMargin: -28
                antialiasing: true
                Layout.fillWidth: true
                Layout.fillHeight: true

                    GroupBox {
                        id: groupBox01
                        height: holdRadius.height+40
                        Layout.fillWidth: true

                        clip: false
                        title: qsTr("Speed (m/s)")

                        label: Label {
                                x: groupBox01.leftPadding
                                y: 25
                                width: groupBox01.availableWidth
                                text: groupBox01.title
                                elide: Text.ElideRight
                                background: Rectangle {
                                    y: 0
                                    width: 0
                                    height: 0
                                    color: "transparent"
                                    border.color: "#ffffff"
                                }
                            }

                        background: Rectangle {
                                y: 0
                                width: 0
                                height: 0
                                color: "transparent"
                                border.color: "#ffffff"
                            }

                        TextField {
                            id: speedText
                            objectName: "moveToRadiusText"
                            placeholderText: qsTr("Insert speed")
                            enabled: true
                        }
                    }

                    GroupBox {
                    id: groupBox02
                    height: holdRadius.height+40
                    Layout.fillWidth: true

                    clip: false
                    title: qsTr("Heading (deg)")

                    label: Label {
                            x: groupBox02.leftPadding
                            y: 25
                            width: groupBox02.availableWidth
                            text: groupBox02.title
                            elide: Text.ElideRight
                            background: Rectangle {
                                y: 0
                                width: 0
                                height: 0
                                color: "transparent"
                                border.color: "#ffffff"
                            }
                        }

                    background: Rectangle {
                            y: 0
                            width: 0
                            height: 0
                            color: "transparent"
                            border.color: "#ffffff"
                        }

                    TextField {
                        id: headingText
                        objectName: "moveToRadiusText"
                        placeholderText: qsTr("Insert speed")
                        enabled: true
                    }
                }
            }

            Button {
                id: speedHeadButton
                y: 42
                text: "Speed-Heading"
                anchors.right: parent.right
                anchors.rightMargin: 0
                anchors.left: parent.left
                anchors.leftMargin: 0
                anchors.bottom: parent.bottom
                anchors.bottomMargin: 0
                antialiasing: false
                Layout.fillWidth: true
                Layout.fillHeight: false


            highlighted: true
            Material.background: pressed? mainAccentColor : Material.color(Material.Blue, Material.Shade700)

            onClicked: {
                    if (speedText.text !== '' && headingText.text !== '') {
                        cmdWrapper.sendSpeedHeadingCommand(speedText.text,
                                                           headingText.text)
                    } else {
                        speedHeadingDialog.open()
                    }
                }
            }
        }

        Rectangle {
            clip:true
            id: rectangle1
            width: 200
            height:90
            color: "#ffffff"
            Layout.fillHeight: false
            Layout.fillWidth: true

            GroupBox {
                id: groupBox1
                height: holdRadius.height+40
                anchors.right: parent.right
                anchors.rightMargin: 0
                anchors.left: parent.left
                anchors.leftMargin: 0
                anchors.top: parent.top
                anchors.topMargin: -28
                clip: true
                title: qsTr("Acceptance radius")

                label: Label {
                        x: groupBox1.leftPadding
                        y: 25
                        width: groupBox1.availableWidth
                        text: "Acceptance radius (m)"
                        elide: Text.ElideRight
                        background: Rectangle {
                            y: 0
                            width: 0
                            height: 0
                            color: "transparent"
                            border.color: "#ffffff"
                        }
                    }

                background: Rectangle {
                        y: 0
                        width: 0
                        height: 0
                        color: "transparent"
                        border.color: "#ffffff"
                    }

                TextField {
                    id: holdRadius
                    objectName: "holdRadiusText"
                    anchors.right: parent.right
                    anchors.rightMargin: 0
                    anchors.left: parent.left
                    anchors.leftMargin: 0
                    placeholderText: qsTr("Insert acceptance radius")
                    enabled: true
                }
            }
            Button {
                id: holdButton
                y: 32
                text: "Hold Position"
                anchors.bottomMargin: 0
                anchors.right: parent.right
                anchors.rightMargin: 0
                anchors.left: parent.left
                anchors.leftMargin: 0
                anchors.bottom: parent.bottom
                antialiasing: false

            highlighted: true
            Material.background: pressed? mainAccentColor : Material.color(Material.Blue, Material.Shade700)

                onClicked: {
                    if (holdRadius.text !== '') {
                        cmdWrapper.sendHoldCommand(parseFloat(holdRadius.text))
                    } else {
                        acceptRadDialog.open()
                    }
                }
            }
        }


        Rectangle {
            clip:true
            id: rectangle2
            width: 200
            height:90
            color: "#ffffff"
            Layout.fillHeight: false
            Layout.fillWidth: true

            GroupBox {
                id: groupBox2
                height: holdRadius.height+40
                anchors.right: parent.right
                anchors.rightMargin: 0
                anchors.left: parent.left
                anchors.leftMargin: 0
                anchors.top: parent.top
                anchors.topMargin: -28
                clip: true
                title: qsTr("Acceptance radius")

                label: Label {
                        x: groupBox2.leftPadding
                        y: 25
                        width: groupBox2.availableWidth
                        text: "Acceptance radius (m)"
                        elide: Text.ElideRight
                        background: Rectangle {
                            y: 0
                            width: 0
                            height: 0
                            color: "transparent"
                            border.color: "#ffffff"
                        }
                    }

                background: Rectangle {
                        y: 0
                        width: 0
                        height: 0
                        color: "transparent"
                        border.color: "#ffffff"
                    }

                TextField {
                    id: moveToRadius
                    objectName: "moveToRadiusText"
                    anchors.right: parent.right
                    anchors.rightMargin: 0
                    anchors.left: parent.left
                    anchors.leftMargin: 0
                    placeholderText: qsTr("Insert acceptance radius")
                    enabled: true
                }
            }
            Button {
                id: moveToButton
                y: 32
                text: "Move to marker"
                anchors.bottomMargin: 0
                anchors.right: parent.right
                anchors.rightMargin: 0
                anchors.left: parent.left
                anchors.leftMargin: 0
                anchors.bottom: parent.bottom
                antialiasing: false
                Material.background: pressed? mainAccentColor : Material.color(Material.Blue, Material.Shade700)
                enabled: Helper.coord_inside_polygon(map.marker_coords, map.polysec_cur.path) && (map.markerIcon.opacity > 0)
                highlighted: true

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
        }

        RowLayout {
            id: rectid
            RowLayout {
                id: loadSavePath
                Layout.fillWidth: true
                Button {
                    id: savePath
                    Layout.fillWidth: true
                    Layout.fillHeight: false
                    text: "Save Path"

                    highlighted: true
                    Material.background: pressed? mainAccentColor : Material.color(Material.Blue, Material.Shade700)

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
                    Layout.fillHeight: false
                    text: "Load Path"

                    highlighted: true
                    Material.background: pressed? mainAccentColor : Material.color(Material.Blue, Material.Shade700)

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
                Layout.fillHeight: false

                highlighted: true
                Material.background:  Material.color(Material.Green, Material.Shade700)

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
