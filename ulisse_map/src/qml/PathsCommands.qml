import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtQml.Models 2.1
import QtQuick.Controls.Material 2.1

import "."

RowLayout {

    property alias pathButtonsColumn: pathButtonsColumn
    property real fontSize: 14
    property real pathButtonWidthRatio: 0.6
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
                    //window.sig_escape.connect(cancelPathCreation)
                    //map.mapTextOverlay.text = "Press ESC to cancel"
                    //map.mapTextOverlay.visible = true
                }
            }

            Button {
                id: deleteTracks
                visible: pathButtonsColumn.children.length > 0
                enabled: true
                text: "Delete Paths..."
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
            width: parent.fillWidth
            visible: false
            Button {
                id: abort
                text: "Back"
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
            Layout.preferredWidth: parent.width * pathButtonWidthRatio

            /*onVisibleChildrenChanged: {
                console.log("bar_manage.n: " + bar_manage.n)
            }*/

            //width: 160
            //y: 96
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
                    if (pathButtonsColumn.children.length === 0) {
                        toast.show("There is nothing to save!")
                        return
                    } else {
                        savePathDialog.open()
                    }
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


    function cancelPathCreation() {
        deselect_all()
        enableBtns(true)
        bar_manage.hide_all()
        bar_manage.discard()
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
            pathButtonsColumn.children[i].managedPath.check_safe(map.safety_polygon)
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
                c.highlight(poly === c.managedPath)
            }
        } else {
            for (i = 0; i < pathButtonsColumn.children.length; i++) {
                c = pathButtonsColumn.children[i]
                if (poly === c.managedPath)
                    c.toggle()
            }
        }
    }

    function delete_item(c){
        c.managedPath.deregister_map_items()
        map.removeMapItem(c.managedPath)
        c.destroy()
        //bar_manage.n--
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

    /*Keys.onEscapePressed: {
         console.log("escapeItem in PathsCommands.qml is handling escape");
        deselect_all()
    }*/
}
