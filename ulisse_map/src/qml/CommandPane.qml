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
                Layout.alignment: Qt.AlignVCenter //| Qt.AlignVCenter
                //Layout.bottomMargin: 20
                font.pointSize: 11
                //Layout.topMargin: 20
                font.weight: Font.DemiBold
            }

            RowLayout {
                id: boundingBoxButtons
                Layout.fillWidth: true

                /*Button {
                    id: buttonBoundBoxGet
                    text: qsTr("Get")
                    font.pointSize: 9
                    padding: 5
                    antialiasing: false
                    Layout.fillWidth: true
                    highlighted: true
                    Material.background: green
                    //Layout.fillHeight: false
                    onClicked: {
                        cmdWrapper.sendBoundaries(JSON.stringify(map.safety_polygon.serialize()))
                    }
                }*/

                Button {
                    id: buttonBoundBoxDefine
                    text: qsTr("Define")
                    font.pointSize: 9
                    padding: 5
                    antialiasing: false
                    Layout.fillWidth: true
                    //Layout.fillHeight: false
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
                    text: qsTr("Resend")
                    font.pointSize: 9
                    padding: 5
                    antialiasing: false
                    Layout.fillWidth: true
                    highlighted: true
                    //Layout.fillHeight: false
                    onClicked: cmdWrapper.sendBoundaries(JSON.stringify(map.safety_polygon.serialize()))
                }

                Button {
                    id: buttonBoundBoxLoad
                    text: "\uf4c2" // icon-folder
                    //text: "\uE802" // load-icon
                    font.family: "fontello"
                    font.pointSize: 10
                    padding: 5
                    //antialiasing: false
                    Layout.maximumWidth: 30
                    highlighted: true
                    Material.background: grey
                    //Layout.fillHeight: false
                    onClicked: {
                        loadPathDialog.open()
                    }
                }



                Button {
                    id: buttonBoundBoxSave
                    text: "\uE816" // save-icon
                    font.family: "fontello"
                    font.pointSize: 10
                    padding: 5
                    //antialiasing: false
                    Layout.maximumWidth: 30
                    highlighted: true
                    Material.background: grey
                    //Layout.fillHeight: false
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
            Material.background: Material.color(Material.BlueGrey, Material.Shade50)
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
        title: "Please choose a file"
        folder: shortcuts.home
        nameFilters: ["Path Files (*.path)"]

        onAccepted: {
            var path = loadPathDialog.fileUrl.toString()
            path = path.replace(/^(file:\/{2})/, "")
            loadPath(path)
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


    /*function savePaths(filePath) {

        var all_paths = {
            //security_box: null,
            paths//TODO security box
            : []
        }

        for (var i = 0; i < pathCmdPane.pathButtonsColumn.children.length; i++) {
            all_paths.paths.push(pathCmdPane.pathButtonsColumn.children[i].managedPath.serialize())
        }

        cmdWrapper.savePathToFile(filePath, JSON.stringify(all_paths))
        toast.show("Path saved")
    }*/

    function savePath(pathToSave, directory) {

        var serializedPath = pathToSave.serialize();
        cmdWrapper.savePathToFile(directory, JSON.stringify(serializedPath))
        toast.show("Path saved")
    }

    function loadPath(filePath) {
        var jsondata = cmdWrapper.loadPathFromFile(filePath)
        var data = JSON.parse(jsondata)

        var i, j, lat, lon, p
        var cur_managed

        //for (i = 0; i < data.paths.length; i++) {
            switch (data.type) {
            case "PolyPath":
                cur_managed = map.createPolySweepPath()
                cur_managed.deserialize(data)
                cur_managed.type = data.type
                var v = pathButtonComponent.createObject(pathCmdPane.pathButtonsColumn)
                v.managedPath = cur_managed
                //v.ntrack = pathCmdPane.pathButtonsColumn.children.length
                v.selected.connect(function (path) {
                    pathCmdPane.update_selection(path)
                    bar_manage.manage(path)
                })
                //console.log("[Command Pane] loadPath - check_safe()")
                cur_managed.check_safe(map.safety_polygon)
                console.log("[Command Pane] loadPath - generate_and_draw_deferred()")
                cur_managed.generate_and_draw_deferred()
                break
            case "PointPath":
                cur_managed = map.createPolylinePath()
                cur_managed.deserialize(data)
                cur_managed.type = data.type
                var v = pathButtonComponent.createObject(pathCmdPane.pathButtonsColumn)
                v.managedPath = cur_managed
                //v.ntrack = pathCmdPane.pathButtonsColumn.children.length
                v.selected.connect(function (path) {
                    pathCmdPane.update_selection(path)
                    bar_manage.manage(path)
                })
                cur_managed.check_safe(map.safety_polygon)
                cur_managed.generate_and_draw_deferred()
                break
            case "SafetyBoundary":
                map.safety_polygon.clear_path()
                map.safety_polygon.deserialize(data)
                map.safety_polygon.close_polygon()
                buttonBoundBoxDefine.end()
                settings.savedBoundary = JSON.stringify(map.safety_polygon.serialize())
                break
            default:
                toast.show("File was not a path", 4000)
                return
            }
        //}
        toast.show("Path loaded", 4000)
    }
}
