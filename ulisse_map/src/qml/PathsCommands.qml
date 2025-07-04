import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtQml.Models 2.1
import QtQuick.Controls.Material 2.1

import "."

RowLayout {

    property alias pathButtonsColumn: pathButtonsColumn
    property real fontSize: 14
    //property bool multichoice: false

    ColumnLayout {
        Layout.fillWidth: true
        Layout.fillHeight: true
        //height: main_btns.height

        RowLayout {
            id: main_btns
            Layout.fillWidth: true

            Button {
                id: addTracks
                highlighted: true
                Material.background: pressed ? orange : softorange
                Layout.fillWidth: true
                Layout.fillHeight: false
                text: qsTr("Define Path...")

                onClicked: function () {
                    bar_manage.show_shape_choice()
                    enableBtns(false)
                    //window.sig_escape.connect(cancelPathCreation)
                    map.mapHintsOverlay.text = "<i>(Press ESC to cancel)</i>"
                    map.mapHintsOverlay.visible = true
                }
            }


            Button {
                id: loadPathButton
                text: "\uf4c2" // icon-folder
                font.family: "fontello"
                font.pointSize: 10
                padding: 5
                Layout.preferredWidth: 50
                highlighted: true
                Material.background: pressed ? orange : softorange
                ToolTip.delay: 1000
                ToolTip.timeout: 5000
                ToolTip.visible: hovered
                ToolTip.text: qsTr("Load Path")
                onClicked: {
                    loadPathDialog.loadType = MapView.PolygonType.Path;
                    loadPathDialog.open = true;
                }
            }
        }

        ScrollView {
            //width: parent.width
            //height : parent.height
            Layout.fillWidth: true
            Layout.fillHeight: true
            contentWidth: parent.width    // The important part
            contentHeight: pathButtonsColumn.height  // Same
            clip : true                   // Prevent drawing column outside the scrollview borders
            ScrollBar.vertical.policy: ScrollBar.AlwaysOn

            Label {
                height: 40
                width: parent.width - 20
                text: "Load or define a path"
                font.pointSize: 14
                font.weight: Font.Light
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                color: lightgrey
                visible: pathButtonsColumn.children.length === 0
            }

            Column {
                id: pathButtonsColumn
                width: parent.width - 22  // space subtracted to avoid overlap with scrolling bar
            }
        }



        //Column {
        //    // Space for the Paths buttons
        //    id: pathButtonsColumn
        //    Layout.fillWidth: true
        //    //width: parent.width
        //}

    }


    function cancelPathCreation() {
        deselect_all()
        enableBtns(true)
        bar_manage.hide_all()
        bar_manage.discard()
    }

    function enableBtns(y) {
        addTracks.enabled = y
        loadPathButton.enabled = y ///FIX
    }

    function restoreBtns() {
        main_btns.visible = true
        //confirm_btns.visible = false
        enableBtns(true)
        //multichoice = false
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
        //if (!multichoice) {
        for (var i = 0; i < pathButtonsColumn.children.length; i++) {
            var c = pathButtonsColumn.children[i]
            c.highlight(poly === c.managedPath)
        }
        /*} else {
            for (i = 0; i < pathButtonsColumn.children.length; i++) {
                c = pathButtonsColumn.children[i]
                if (poly === c.managedPath)
                    c.toggle()
            }
        }*/
    }

    /*function delete_item(c){
        c.managedPath.deregister_map_items()
        map.removeMapItem(c.managedPath)
        c.destroy()
        //bar_manage.n--
    }*/

    /*function delete_item_at_idx(idx){
        var c = pathButtonsColumn.children[idx]
        delete_item(c)
    }*/

    /*function delete_all() {
        for (var i = pathButtonsColumn.children.length - 1; i >= 0; i--)
            delete_item_at_idx(i)
    }*/

    /*function delete_toggled_items() {
        bar_manage.hide_all()
        for (var i = pathButtonsColumn.children.length - 1; i >= 0; i--) {
            var c = pathButtonsColumn.children[i]
            if (c.toggled) {
                delete_item_at_idx(i)
            }
        }
        restoreBtns()
        deselect_all()

    }*/

    /*Keys.onEscapePressed: {
         console.log("escapeItem in PathsCommands.qml is handling escape");
        deselect_all()
    }*/
}
