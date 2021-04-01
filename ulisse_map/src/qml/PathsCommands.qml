import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtQml.Models 2.1
import QtQuick.Controls.Material 2.1
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
                Material.background: pressed ? orange : mainColor
                Layout.fillWidth: true
                Layout.fillHeight: false
                text: qsTr("Add Path...")

                onClicked: function () {
                    bar_manage.show_shape_choice()
                    enableBtns(false)
                }

                /*background: Rectangle {
                        id: addTracksRect
                        color: blue
                        anchors.fill: parent
                        Image {
                            id: addimg
                            anchors.horizontalCenter: parent.horizontalCenter
                            anchors.verticalCenter: parent.verticalCenter
                            visible: sliderTogglerLeft.checked ? false : true
                            source: 'qrc:/images/add-path.png'
                        }
                    }*/
            }

            Button {
                id: deleteTracks
                visible: pathButtonsColumn.children.length > 0
                enabled: true
                text: "Delete Paths"
                Layout.fillWidth: true
                Layout.fillHeight: false
                highlighted: true
                Material.background: pressed ? orange : mainColor

                onClicked: function () {
                    multichoice = true
                    bar_manage.inhibit = true
                    bar_manage.hide_all()
                    sweepPathCmdPane.deselect_all()
                    main_btns.visible = false
                    confirm_btns.visible = true
                }

                /*background: Rectangle {
                        color: blue
                        anchors.fill: parent
                        id: trackBG
                        Image {
                            id: trashimg
                            anchors.horizontalCenter: parent.horizontalCenter
                            anchors.verticalCenter: parent.verticalCenter
                            visible: sliderTogglerLeft.checked ? false : true
                            source: 'qrc:/images/trash.png'
                        }
                    }*/
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

                /*background: Rectangle {
                        id: closeimg
                        Image {
                            visible: sliderTogglerLeft.checked ? false : true
                            anchors.horizontalCenter: parent.horizontalCenter
                            anchors.verticalCenter: parent.verticalCenter
                            source: 'qrc:/images/close.png'
                        }
                        color: red
                    }*/
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

                /*background: Rectangle {
                        color: green
                        id: checkimg
                        Image {
                            visible: sliderTogglerLeft.checked ? false : true
                            anchors.horizontalCenter: parent.horizontalCenter
                            anchors.verticalCenter: parent.verticalCenter
                            source: 'qrc:/images/check.png'
                        }
                    }*/
            }
        }

        Column {
            // Space for the Paths buttons
            id: pathButtonsColumn
            width: defheigth + 120
            y: 96
        }
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
