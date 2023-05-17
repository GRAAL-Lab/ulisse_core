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

    property var safetyPoly_bkp: []
    property var buttonSafety: buttonBoundBoxDefine
    property var pathButtonComponent
    property alias speedHeadTimeout: commandParamsStackContainer.speedHeadTimeout
    property alias pathCmdPane: commandParamsStackContainer.pathCommandsPane
    property var sliderHeading: commandParamsStackContainer.sliderHeading
    property alias savePathDialog: savePathDialog

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
                Layout.alignment: Qt.AlignVCenter
                font.pointSize: 11
                font.weight: Font.DemiBold
            }

            RowLayout {
                id: boundingBoxButtons
                Layout.fillWidth: true

                Button {
                    id: buttonBoundBoxDefine
                    text: qsTr("Define")
                    font.pointSize: 9
                    padding: 5
                    antialiasing: false
                    Layout.fillWidth: true
                    highlighted: true
                    Material.background: green

                    onClicked: {
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
                        settings.savedBoundary = JSON.stringify(map.safety_polygon.serialize())
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
                    enabled: settings.savedBoundary == "null" ? false : true
                    text: fbkUpdater.safety_boundary_set? qsTr("Resend") : qsTr("Send")
                    font.pointSize: 9
                    padding: 5
                    antialiasing: false
                    Layout.fillWidth: true
                    highlighted: true
                    Material.background: fbkUpdater.safety_boundary_set? grey : softorange
                    onClicked: cmdWrapper.sendBoundaries(JSON.stringify(map.safety_polygon.serialize()))
                }

                Button {
                    id: buttonBoundBoxLoad
                    text: "\uf4c2" // icon-folder
                    font.family: "fontello"
                    font.pointSize: 10
                    padding: 5
                    Layout.maximumWidth: 30
                    highlighted: true
                    Material.background: grey
                    onClicked: {
                        loadPathDialog.loadType = MapView.PolygonType.SafetyArea;
                        loadPathDialog.open()
                    }
                }


                Button {
                    id: buttonBoundBoxSave
                    text: "\uE816" // save-icon
                    font.family: "fontello"
                    font.pointSize: 10
                    padding: 5
                    Layout.maximumWidth: 30
                    highlighted: true
                    Material.background: grey
                    onClicked: {
                        savePathDialog.pathToSave = map.safety_polygon
                        savePathDialog.open()
                    }
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
            //height: commandsLayout.height
            Material.background: Material.color(Material.BlueGrey, Material.Shade50)

            Layout.alignment:  Qt.AlignVCenter | Qt.AlignHCenter


            Rectangle {
                visible: !fbkUpdater.safety_boundary_set
                Layout.alignment:  Qt.AlignVCenter | Qt.AlignHCenter
                height: commandParamsStackContainer.height
                width: commandParamsStackContainer.width
                color: Qt.hsla(0, 0, 0, 0.25)
                //opacity: 0.5
                z: 99

                layer.enabled: true
                layer.effect: Glow {
                    radius: 5
                    samples: 10
                    color: Qt.hsla(0, 0, 0, 0.25)

                }

                ColumnLayout {

                    anchors.horizontalCenter: parent.horizontalCenter

                    // Spacer Item
                    Item {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 10
                        //Rectangle { anchors.fill: parent; color: "#ffaaaa" } // to visualize the spacer
                    }

                    Image {
                        width: 40
                        Layout.alignment: Qt.AlignCenter
                        source: 'qrc:/images/white_arrow.png'
                        sourceSize.height: 120
                    }

                    Text {

                        Layout.alignment:  Qt.AlignCenter
                        horizontalAlignment: Text.AlignHCenter
                        text: "Boundary not set for the controller! Please ensure that a bounding box has been defined and send it to the catamaran."
                        wrapMode: Text.WordWrap
                        Layout.maximumWidth: commandParamsStackContainer.width - 30

                        color: 'white'
                        z: 100

                        font.weight: Font.Bold

                        layer.enabled: true
                        layer.effect: Glow {
                            radius: 10
                            samples: 10
                            color: grey

                        }
                    }
                }

                MouseArea {
                    anchors.fill: parent
                    propagateComposedEvents: false
                    hoverEnabled: true
                    preventStealing: true
                    onClicked: {
                        overlay.visible = false
                    }
                }
            }
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





    FileDialog {
        id: loadPathDialog

        property int loadType: -1

        title: "Please choose a file"
        folder: shortcuts.home
        nameFilters: ["Path Files (*.path)"]

        onAccepted: {
            var filepath = loadPathDialog.fileUrl.toString()
            filepath = filepath.replace(/^(file:\/{2})/, "")

            switch (loadType) {
            case MapView.PolygonType.SafetyArea:
                loadSafetyArea(filepath)
                break
            case MapView.PolygonType.Path:
                loadPath(filepath)
                break
            default:
                toast.show("Type not supported")
            }
        }
    }

    FileDialog {
        id: savePathDialog
        property var pathToSave

        title: "Saving path..."
        folder: shortcuts.home
        selectExisting: false
        nameFilters: ["Path Files (*.path)"]

        onAccepted: {
            var directory = savePathDialog.fileUrl.toString()
            directory = directory.replace(/^(file:\/{2})/, "") // remove prefixed "file://"
            directory = decodeURIComponent(directory) // unescape html codes like '%23' for '#'
            savePath(pathToSave, directory)
        }
    }

    function savePath(pathToSave, directory) {

        var serializedPath = pathToSave.serialize();
        addonsBridge.savePathToFile(directory, JSON.stringify(serializedPath))
        toast.show("Path saved")
    }

    function loadSafetyArea(filePath) {
        var jsondata = addonsBridge.loadPathFromFile(filePath)
        var data = JSON.parse(jsondata)

        switch (data.type) {
        case "SafetyBoundary":
            map.safety_polygon.clear_path()
            map.safety_polygon.deserialize(data)
            map.safety_polygon.close_polygon()
            buttonBoundBoxDefine.end()
            settings.savedBoundary = JSON.stringify(map.safety_polygon.serialize())
            break
        default:
            toast.show("File was not a safety area", 4000)
            return
        }

        toast.show("Safety Area loaded", 4000)
    }

    function loadPath(filePath) {
        var jsondata = addonsBridge.loadPathFromFile(filePath)
        var data = JSON.parse(jsondata)

        var i, j, lat, lon, p
        var cur_managed, pathButton

        switch (data.type) {
        case "PolyPath":
            cur_managed = map.createPolySweepPath()
            cur_managed.deserialize(data)
            cur_managed.type = data.type
            pathButton = pathButtonComponent.createObject(pathCmdPane.pathButtonsColumn)
            pathButton.managedPath = cur_managed
            pathButton.selected.connect(function (path) {
                pathCmdPane.update_selection(path)
                bar_manage.manage(path)
            })
            cur_managed.check_safe(map.safety_polygon)
            //console.log("[Command Pane] loadPath - generate_and_draw_deferred()")
            cur_managed.generate_and_draw_deferred()
            break
        case "PointPath":
            cur_managed = map.createPolylinePath()
            cur_managed.deserialize(data)
            cur_managed.type = data.type
            pathButton = pathButtonComponent.createObject(pathCmdPane.pathButtonsColumn)
            pathButton.managedPath = cur_managed
            pathButton.selected.connect(function (path) {
                pathCmdPane.update_selection(path)
                bar_manage.manage(path)
            })
            cur_managed.check_safe(map.safety_polygon)
            cur_managed.generate_and_draw_deferred()
            break
        default:
            toast.show("File was not a path", 4000)
            return
        }
        pathButton.selected(pathButton.managedPath)
        toast.show("Path loaded", 4000)
    }



}
