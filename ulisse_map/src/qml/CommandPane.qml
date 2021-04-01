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

    property var polysec_bkp: []
    property var buttonSafety: buttonBoundBoxDefine
    property var trackComponent
    property alias speedHeadTimeout: speedHeadTimeout
    property alias buttonSafety1: buttonBoundBoxResend
    property alias sweepPathCmdPane: sweepPathCmdPane
    visible: true

    Layout.fillWidth: true
    Layout.fillHeight: true

    Component.onCompleted: {
        trackComponent = Qt.createComponent("PathButton.qml")
    }

    ColumnLayout {
        id: controlsColumn
        anchors.fill: parent
        spacing: 0

        RowLayout {
            id: boundingBoxControls
            Layout.fillWidth: true

            Label {
                color: green
                text: "Safety area"
                antialiasing: false
                //Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                Layout.bottomMargin: 10
                font.pointSize: 12
                Layout.topMargin: 20
                font.weight: Font.DemiBold
            }

            RowLayout {
                id: boundingBoxButtons
                Layout.fillWidth: true
                Button {
                    id: buttonBoundBoxDefine
                    text: qsTr("Define")

                    padding: 5
                    antialiasing: false
                    Layout.fillWidth: true
                    Layout.fillHeight: false

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
                        sweepPathCmdPane.check_safety_all()
                        cmdWrapper.sendBoundaries(JSON.stringify(map.polysec_cur.serialize()))
                    }

                    function reset_polysec(){
                        enabled = true
                        map.polysec_cur.end.disconnect(end)
                        window.sig_escape.disconnect(reset_polysec)
                        map.polysec_cur.path = polysec_bkp
                        map.click_handler = map.click_goto_handler
                        map.pos_changed_handler = function () {}
                    }

                    onClicked: function () {
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
                    padding: 5
                    antialiasing: false
                    Layout.fillWidth: true
                    highlighted: true
                    Layout.fillHeight: false
                    onClicked: cmdWrapper.sendBoundaries(JSON.stringify(map.polysec_cur.serialize()))
                }
            }
        }

        Label {
            text: "Parameters"
            color: green
            Layout.bottomMargin: 10
            Layout.topMargin: 5
            antialiasing: false
            font.weight: Font.DemiBold
            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
            font.pointSize: 12
        }

        RowLayout {
            Layout.fillWidth: true
            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter

            Label {
                text: qsTr("Cruise speed")
                clip: false
                leftPadding: 5
                font.pointSize: 11
                Layout.fillWidth: true
                verticalAlignment: Text.AlignVCenter
                horizontalAlignment: Text.AlignLeft
            }

            Slider {
                id: sliderSpeed
                objectName: "cruiseSpeed"
                Layout.preferredWidth: 110
                Layout.leftMargin: 0
                to: 5
                from: 0.0
                stepSize: 0.1
                value: 1
                onPressedChanged: function() {
                    if (sliderSpeed.pressed === false)
                        cmdWrapper.setCruiseSpeedCommand(sliderSpeed.value)
                }
            }

            Text {
                text: sliderSpeed.value.toFixed(1) + " m/s"
                verticalAlignment: Text.AlignVCenter
                horizontalAlignment: Text.AlignLeft
                Layout.leftMargin: 10
                Layout.fillWidth: true
                font.pointSize: 11
            }
        }


        RowLayout {
            Layout.fillWidth: true
            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter

            Label {
                text: qsTr("Accept. radius")
                leftPadding: 5
                font.pointSize: 11
                Layout.fillWidth: true
                horizontalAlignment: Text.AlignLeft
                verticalAlignment: Text.AlignVCenter
            }

            Slider {
                id: holdRadius
                Layout.preferredWidth: 110
                Layout.leftMargin: 0
                value: 5
                stepSize: 0.1
                from: 0.5
                to: 10
            }

            Text {
                text: holdRadius.value.toFixed(1) + " m"
                Layout.fillWidth: true
                font.pointSize: 11
                horizontalAlignment: Text.AlignLeft
                verticalAlignment: Text.AlignVCenter
            }
        }


        RowLayout {
            Layout.fillWidth: true
            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter

            Label {
                text: qsTr("Heading")
                leftPadding: 5
                font.pointSize: 11
                Layout.fillWidth: true
                horizontalAlignment: Text.AlignLeft
                verticalAlignment: Text.AlignBottom
            }

            Slider {
                id: sliderHeading
                //width: 250
                //wheelEnabled: false
                value: 5
                Layout.preferredWidth: 110
                Layout.leftMargin: 0
                stepSize: 0.1
                from: 0
                to: 359.9
            }

            Text {
                width: 80
                text: sliderHeading.value.toFixed(1) + " °"
                font.pointSize: 11
                horizontalAlignment: Text.AlignLeft
                verticalAlignment: Text.AlignVCenter
            }
        }

        RowLayout {
            Layout.fillWidth: true
            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter

            Label {
                text: qsTr("Duration (0 = indefinite)")
                font.underline: false
                leftPadding: 5
                font.pointSize: 11
                horizontalAlignment: Text.AlignLeft
                Layout.fillWidth: true
                verticalAlignment: Text.AlignBottom
            }

            SpinBox {
                id: speedHeadTimeout
                objectName: "shTimeout"
                editable: true
                to: 1000
                value: 200
                Layout.fillWidth: true
                onValueChanged: function(){
                    settings.shTimeout = speedHeadTimeout.value
                }
            }

            Text {
                id: element3
                text: "s"
                font.pointSize: 11
                horizontalAlignment: Text.AlignLeft
                verticalAlignment: Text.AlignVCenter
            }
        }


        Label {
            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
            font.pointSize: 11
            font.weight: Font.DemiBold
            color: green
            text: "Commands"
            //Layout.bottomMargin: 10
            //Layout.topMargin: 20
            antialiasing: false
        }

        Button {
            id: speedHeadButton
            y: 42
            text: "Speed/Heading"
            /*anchors.right: parent.right
                anchors.rightMargin: 0
                anchors.left: parent.left
                anchors.leftMargin: 0
                anchors.bottom: parent.bottom
                anchors.bottomMargin: 0*/
            antialiasing: false
            Layout.fillWidth: true
            Layout.fillHeight: false

            highlighted: true
            Material.background: pressed ? orange : mainColor

            onClicked: {
                cmdWrapper.sendSpeedHeadingCommand(sliderSpeed.value,
                                                   sliderHeading.value)
            }
        }


    Button {
        width: parent.width
        text: "Halt"
        antialiasing: false
        Layout.fillWidth: true

        highlighted: true
        Material.background: pressed ? orange : mainColor

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
        Material.background: pressed ? orange : mainColor

        onClicked: {

            cmdWrapper.sendHoldCommand(holdRadius.value)
        }
    }

    Button {
        id: moveToButton
        text: "Move to marker"
        Layout.fillWidth: true
        antialiasing: false
        Material.background: pressed ? orange : mainColor
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
        id: pathlabel
        color: green
        text: "Paths"
        //Layout.bottomMargin: 10
        Layout.topMargin: 10
        antialiasing: false
        font.weight: Font.DemiBold
        Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
        font.pointSize: 12
    }

    PathsCommands {
        id: sweepPathCmdPane
        Layout.bottomMargin: 10
        //Layout.topMargin: 20
        Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
        Layout.fillWidth: true
        Layout.fillHeight: false

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
                text: "Save"

                highlighted: true
                Material.background: pressed ? orange : mainColor

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
                text: "Load"

                highlighted: true
                Material.background: pressed ? orange : mainColor

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
    if (sweepPathCmdPane.columnTrack.children.length === 0) {
        toast.show("There is nothing to save!")
        return
    }

    var all_paths = {
        security_box: null,
        paths//TODO security box
        : []
    }

    for (var i = 0; i < sweepPathCmdPane.columnTrack.children.length; i++) {
        all_paths.paths.push(
                    sweepPathCmdPane.columnTrack.children[i].managed_path.serialize())
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

            var v = trackComponent.createObject(sweepPathCmdPane.columnTrack)
            v.managed_path = cur_managed
            v.ntrack = sweepPathCmdPane.columnTrack.children.length
            v.selected.connect(function (path) {
                sweepPathCmdPane.update_selection(path)
                bar_manage.manage(path)
            })

            cur_managed.draw_deferred()
            break
        case "PointPath":
            var cur_managed = map.createPath()
            cur_managed.deserialize(data.paths[i])

            var v = trackComponent.createObject(sweepPathCmdPane.columnTrack)
            v.managed_path = cur_managed
            v.ntrack = sweepPathCmdPane.columnTrack.children.length
            v.selected.connect(function (path) {
                sweepPathCmdPane.update_selection(path)
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
    D{i:0;height:1078;width:300}D{i:5;anchors_x:0}D{i:4;anchors_x:0}D{i:11;anchors_x:0}
D{i:10;anchors_x:0}D{i:20;anchors_x:0}D{i:19;anchors_x:0}D{i:24;anchors_width:200;anchors_x:0}
}
##^##*/
