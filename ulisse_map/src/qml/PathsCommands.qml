import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtQml.Models 2.1
import QtQuick.Controls.Material 2.1
import QtQuick.Dialogs 1.2
import "."

RowLayout {

    property alias columnTrack: pathButtonsColumn
    property real fontSize: 14
    property int defheigth: 42
    property bool multichoice: false

    ColumnLayout {
        id: sweepCommandsContainer
        //spacing: 10
        Layout.fillWidth: true
        height: main_btns.height

        RowLayout {
            id: main_btns
            Layout.fillWidth: true

            Button {
                id: addTracks
                highlighted: true
                Material.background: pressed ? orange : softorange
                Layout.fillWidth: true
                Layout.fillHeight: false
                text: qsTr("Add Path...")

                onClicked: function () {
                    bar_manage.show_shape_choice()
                    enableBtns(false)
                }
            }

            Button {
                id: deleteTracks
                visible: pathButtonsColumn.children.length > 0
                enabled: true
                text: "Delete Paths"
                Layout.fillWidth: true
                Layout.fillHeight: false
                highlighted: true
                Material.background: pressed ? orange : softorange

                onClicked: function () {
                    multichoice = true
                    bar_manage.inhibit = true
                    bar_manage.hide_all()
                    pathCommandsPane.deselect_all()
                    main_btns.visible = false
                    confirm_btns.visible = true
                }
            }
        }

        RowLayout {
            id: confirm_btns
            width: defheigth + 120
            visible: false
            Button {
                id: abort
                text:"Back"
                Layout.fillWidth: true
                Layout.fillHeight: false
                highlighted: true
                Material.background: green

                onClicked: function () {
                    restoreBtns()
                    deselect_all()
                    enableBtns(true)
                    bar_manage.inhibit = false
                    bar_manage.hide_all()
                    bar_manage.show_manage()
                }
            }
            Button {
                id: confirm
                text: "Delete"
                enabled: true
                Layout.fillWidth: true
                Layout.fillHeight: false
                highlighted: true
                Material.background: red

                onClicked: function(){
                    delete_toggled_items()
                    bar_manage.inhibit = false
                }
            }
        }

        Column {
            // Space for the Paths buttons
            id: pathButtonsColumn
            width: defheigth + 120
            y: 96
        }


        RowLayout {
            id: loadSavePath
            Layout.fillWidth: true
            Button {
                id: savePath
                Layout.fillWidth: true
                Layout.fillHeight: false
                text: "Save"

                highlighted: true
                Material.background: pressed ? orange : softorange

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
                Material.background: pressed ? orange : softorange

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

    FileDialog {
        id: loadPathDialog
        title: "Please choose a file"
        folder: shortcuts.home
        nameFilters: ["Path Files (*.path)"]

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
        nameFilters: ["Path Files (*.path)"]

        onAccepted: {
            var path = savePathDialog.fileUrl.toString()
            path = path.replace(/^(file:\/{2})/,
                                "") // remove prefixed "file://"
            path = decodeURIComponent(
                        path) + ".path" // unescape html codes like '%23' for '#'
            savePaths(path)
        }
    }

    function savePaths(filePath) {
        if (pathCommandsPane.columnTrack.children.length === 0) {
            toast.show("There is nothing to save!")
            return
        }

        var all_paths = {
            security_box: null,
            paths//TODO security box
            : []
        }

        for (var i = 0; i < pathCommandsPane.columnTrack.children.length; i++) {
            all_paths.paths.push(
                        pathCommandsPane.columnTrack.children[i].managed_path.serialize())
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

                var v = trackComponent.createObject(pathCommandsPane.columnTrack)
                v.managed_path = cur_managed
                v.ntrack = pathCommandsPane.columnTrack.children.length
                v.selected.connect(function (path) {
                    pathCommandsPane.update_selection(path)
                    bar_manage.manage(path)
                })

                cur_managed.draw_deferred()
                break
            case "PointPath":
                var cur_managed = map.createPath()
                cur_managed.deserialize(data.paths[i])

                var v = trackComponent.createObject(pathCommandsPane.columnTrack)
                v.managed_path = cur_managed
                v.ntrack = pathCommandsPane.columnTrack.children.length
                v.selected.connect(function (path) {
                    pathCommandsPane.update_selection(path)
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




    function enableBtns(y) {
        addTracks.enabled = y
        deleteTracks.enabled = y
    }

    function restoreBtns() {
        main_btns.visible = true
        confirm_btns.visible = false
        enableBtns(true)
        multichoice = false
    }

    function check_safety_all() {
        for (var i = 0; i < pathButtonsColumn.children.length; i++) {
            pathButtonsColumn.children[i].managed_path.check_safe(map.polysec_cur)
        }
    }

    function deselect_all() {
        for (var i = 0; i < pathButtonsColumn.children.length; i++) {
            var c = pathButtonsColumn.children[i]
            c.highlight(false)
        }
    }
    function update_selection(poly) {
        if (!multichoice) {
            for (var i = 0; i < pathButtonsColumn.children.length; i++) {
                var c = pathButtonsColumn.children[i]
                c.highlight(poly === c.managed_path)
            }
        } else {
            for (i = 0; i < pathButtonsColumn.children.length; i++) {
                c = pathButtonsColumn.children[i]
                if (poly === c.managed_path)
                    c.toggle()
            }
        }
    }

    function delete_item(c){
        c.managed_path.deregister_map_items()
        map.removeMapItem(c.managed_path)
        c.destroy()
        bar_manage.n--
    }

    function delete_item_at_idx(idx){
        var c = pathButtonsColumn.children[idx]
        delete_item(c)
    }

    function delete_all() {
        for (var i = pathButtonsColumn.children.length - 1; i >= 0; i--)
            delete_item_at_idx(i)
    }

    function delete_toggled_items() {
        bar_manage.hide_all()
        for (var i = pathButtonsColumn.children.length - 1; i >= 0; i--) {
            var c = pathButtonsColumn.children[i]
            if (c.toggled) {
                delete_item_at_idx(i)
            }
        }
        restoreBtns()
        deselect_all()
    }
}
