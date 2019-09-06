import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.2
import "."
import "../scripts/helper.js" as Helper

Pane {

    property var buttonSafety: buttonSafety
    property var trackComponent
    visible: true

    Layout.fillWidth: true
    Layout.fillHeight: true

    Component.onCompleted: {
        trackComponent = Qt.createComponent("PathIcon.qml")
    }

    ColumnLayout {
        id: buttonsColumn
        anchors.fill: parent
        spacing: 0

        Label {
            color: green
            text: "Parameters"
            Layout.bottomMargin: 10
            Layout.topMargin: 20
            antialiasing: false
            font.weight: Font.DemiBold
            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
            font.pointSize: 12
        }

        Rectangle {
            id: rectangle1
            width: 200
            height: 70

            color: "#00ffffff"
            Layout.fillWidth: true

            ColumnLayout {
                y: 0
                anchors.right: parent.right
                anchors.rightMargin: 0
                anchors.left: parent.left
                anchors.leftMargin: 0
                id: b1

                Label {
                    id: label1
                    text: qsTr("Cruise speed")
                    font.pointSize: 12
                    Layout.fillWidth: true
                    verticalAlignment: Text.AlignVCenter
                    horizontalAlignment: Text.AlignHCenter
                }

                RowLayout {
                    spacing: 5
                    Layout.topMargin: 0
                    Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                    Layout.fillWidth: true

                    Slider {
                        id: speedText
                        width: 250
                        height: 48
                        Layout.leftMargin: 0
                        Layout.fillWidth: true
                        to: 5
                        from: 0.1
                        stepSize: 0.1
                        value: 1
                    }

                    Text {
                        id: element
                        width: 80
                        height: 48
                        text: speedText.value.toFixed(1) + " m/s"
                        Layout.maximumWidth: 65
                        Layout.minimumWidth: 65
                        Layout.preferredWidth: 65
                        Layout.rightMargin: 0
                        verticalAlignment: Text.AlignVCenter
                        horizontalAlignment: Text.AlignLeft
                        Layout.fillHeight: true
                        Layout.leftMargin: 10
                        Layout.fillWidth: false
                        font.pixelSize: 18
                    }
                }

            }

        }

        Rectangle {
            id: rectangle2
            width: 200
            height: 70
            color: "#00ffffff"
            Layout.fillWidth: true
            ColumnLayout {
                id: b2
                y: 0
                anchors.rightMargin: 0
                anchors.left: parent.left
                anchors.right: parent.right
                Label {
                    id: label2
                    text: qsTr("Acceptance radius")
                    font.pointSize: 12
                    Layout.fillWidth: true
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }

                RowLayout {
                    Layout.fillWidth: true
                    Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                    Slider {
                        id: holdRadius
                        width: 250
                        value: 5
                        Layout.fillWidth: true
                        Layout.leftMargin: 0
                        stepSize: 0.1
                        from: 0.5
                        to: 10
                    }

                    Text {
                        id: element1
                        width: 80
                        text: holdRadius.value.toFixed(1) + " m"
                        Layout.maximumWidth: 65
                        Layout.preferredWidth: 65
                        Layout.minimumWidth: 65
                        Layout.leftMargin: 10
                        Layout.fillWidth: false
                        font.pixelSize: 18
                        Layout.rightMargin: 0
                        horizontalAlignment: Text.AlignLeft
                        Layout.fillHeight: true
                        verticalAlignment: Text.AlignVCenter
                    }
                }
                anchors.leftMargin: 0
            }
        }

        Label {
            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
            font.pointSize: 12
            font.weight: Font.DemiBold
            color: green
            text: "Commands"
            Layout.bottomMargin: 10
            Layout.topMargin: 20
            antialiasing: false
        }

        Rectangle {
            clip: true
            id: rectangle0
            width: 200
            height: 110
            color: "transparent"
            Layout.fillHeight: false
            Layout.fillWidth: true

            RowLayout {
                id: speedHeadingRow
                anchors.top: parent.top
                anchors.right: parent.right
                anchors.rightMargin: 0
                anchors.left: parent.left
                anchors.leftMargin: 0
                anchors.topMargin: 0
                antialiasing: true
                Layout.fillWidth: true
                Layout.fillHeight: true

                Rectangle {
                    id: rectangle3
                    width: 200
                    height: 70
                    color: "#00ffffff"
                    Layout.fillWidth: true
                    ColumnLayout {
                        id: b3
                        y: 0
                        anchors.rightMargin: 0
                        anchors.left: parent.left
                        anchors.right: parent.right
                        Label {
                            id: label3
                            text: qsTr("Heading")
                            font.pointSize: 12
                            Layout.fillWidth: true
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                            Slider {
                                id: headingText
                                width: 250
                                wheelEnabled: false
                                value: 5
                                Layout.leftMargin: 0
                                Layout.fillWidth: true
                                stepSize: 0.1
                                from: 0
                                to: 359.9
                            }

                            Text {
                                id: element2
                                width: 80
                                text: headingText.value.toFixed(1) + " °"
                                Layout.minimumWidth: 65
                                Layout.fillWidth: false
                                Layout.leftMargin: 10
                                Layout.preferredWidth: 65
                                font.pixelSize: 18
                                Layout.maximumWidth: 65
                                Layout.rightMargin: 0
                                horizontalAlignment: Text.AlignLeft
                                Layout.fillHeight: true
                                verticalAlignment: Text.AlignVCenter
                            }
                        }
                        anchors.leftMargin: 0
                    }
                }
            }

            Button {
                id: speedHeadButton
                y: 42
                text: "follow orientation"
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
                Material.background: pressed ? orange : blue

                onClicked: {
                        cmdWrapper.sendSpeedHeadingCommand(speedText.value,
                                                           headingText.value)
                }
            }
        }

        Button {
            width: parent.width
            text: "Halt"
            antialiasing: false
            Layout.fillWidth: true

            highlighted: true
            Material.background: pressed ? orange : blue

            font.capitalization: Font.AllUppercase

            Layout.fillHeight: false

            onClicked: {
                cmdWrapper.sendHaltCommand()
            }
        }



        Button {
            id: holdButton
            text: "Hold Position"
            Layout.fillWidth: true
            antialiasing: false

            highlighted: true
            Material.background: pressed ? orange : blue

            onClicked: {
                    cmdWrapper.sendHoldCommand(holdRadius.value)
            }
        }

        Button {
            id: moveToButton
            text: "Move to marker"
            Layout.fillWidth: true
            antialiasing: false
            Material.background: pressed ? orange : blue
            enabled: Helper.coord_inside_polygon(map.marker_coords,
                                                 map.polysec_cur.path)
                     && (map.markerIcon.opacity > 0)
            highlighted: true

            onClicked: {
                cmdWrapper.sendLatLongCommand(
                                map.marker_coords,
                                holdRadius.value)
            }
        }

        Label {
            color: green
            text: "Planning"
            Layout.bottomMargin: 10
            Layout.topMargin: 20
            antialiasing: false
            font.weight: Font.DemiBold
            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
            font.pointSize: 12
        }

        RowLayout {
            id: rectid
            Layout.fillWidth: true
            RowLayout {
                id: loadSavePath
                Layout.fillWidth: true
                Button {
                    id: savePath
                    Layout.fillWidth: true
                    Layout.fillHeight: false
                    text: "Save Path"

                    highlighted: true
                    Material.background: pressed ? orange : blue

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
                    Material.background: pressed ? orange : blue

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
                Material.background: green

                function end() {
                    enabled = true
                    map.polysec_cur.end.disconnect(end)
                    map.click_handler = map.click_goto_handler
                    map.pos_changed_handler = function () {}
                    text = "Redefine safety area"
                    sidebar_manage.check_safety_all()
                    cmdWrapper.sendBoundaries(JSON.stringify(
                                                  map.polysec_cur.serialize()))
                }
                onClicked: function () {
                    map.polysec_cur.clear_path()
                    map.center = fbkUpdater.ulisse_pos
                    map.click_handler = map.polysec_cur.click_handler
                    map.pos_changed_handler = map.polysec_cur.pos_changed_handler
                    enabled = false
                    map.polysec_cur.end.connect(end)
                }
            }
        }

        Rectangle {
            id: rectangle
            width: 200
            height: 200
            color: "#ffffff"
            Layout.fillHeight: true
            Layout.fillWidth: true
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
            path = path.replace(/^(file:\/{2})/,
                                "") // remove prefixed "file://"
            path = decodeURIComponent(
                        path) // unescape html codes like '%23' for '#'
            savePaths(path)
        }
    }

    function savePaths(filePath) {
        if (sidebar_manage.columnTrack.children.length === 0) {
            toast.show("There is nothing to save!")
            return
        }

        var all_paths = {
            security_box: null,
            paths//TODO security box
            : []
        }

        for (var i = 0; i < sidebar_manage.columnTrack.children.length; i++) {
            all_paths.paths.push(
                        sidebar_manage.columnTrack.children[i].managed_path.serialize())
        }

        cmdWrapper.savePathToFile(filePath, JSON.stringify(all_paths))
        toast.show("Mission saved", 2000)
    }

    function loadPaths(filePath) {
        var jsondata = cmdWrapper.loadPathFromFile(filePath)
        var data = JSON.parse(jsondata)

        var i, j, lat, lon, p

        for (i = 0; i < data.paths.length; i++) {
            switch (data.paths[i].type) {
            case "PolyPath":
                var cur_managed = map.createPoly()
                cur_managed.deserialize(data.paths[i])

                var v = trackComponent.createObject(sidebar_manage.columnTrack)
                v.managed_path = cur_managed
                v.ntrack = sidebar_manage.columnTrack.children.length
                v.selected.connect(function (path) {
                    sidebar_manage.update_selection(path)
                    bar_manage.manage(path)
                })

                cur_managed.draw_deferred()
                break
            case "PointPath":
                var cur_managed = map.createPath()
                cur_managed.deserialize(data.paths[i])

                var v = trackComponent.createObject(sidebar_manage.columnTrack)
                v.managed_path = cur_managed
                v.ntrack = sidebar_manage.columnTrack.children.length
                v.selected.connect(function (path) {
                    sidebar_manage.update_selection(path)
                    bar_manage.manage(path)
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
        toast.show("Mission loaded", 2000)
    }
}

/*##^##
Designer {
    D{i:0;autoSize:true;height:10;width:640}D{i:5;anchors_x:0}D{i:4;anchors_x:0}D{i:11;anchors_x:0}
D{i:10;anchors_x:0}D{i:20;anchors_x:0}D{i:19;anchors_x:0}
}
##^##*/
