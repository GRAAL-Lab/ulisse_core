import QtQuick 2.6
import QtQuick.Controls 1.4 as C1
import QtQuick.Controls.Styles 1.4 as C1S
import QtQuick.Controls 2.2
import QtQml.Models 2.1
import QtQuick.Controls.Material 2.1
import "."

Row {
    id: containerRowLeft

    property alias columnTrack: columnTrack
    property real fontSize: 14
    property int defheigth: 42
    property color labelBackground: "transparent"
    property int edge: Qt.LeftEdge
    property color togglerColor: orange
    property int sliderW: 24
    property bool multichoice: false

    function rightEdge() {
        return (containerRowLeft.edge === Qt.RightEdge)
    }

    layoutDirection: rightEdge() ? Qt.LeftToRight : Qt.RightToLeft
    anchors.top: parent.top
    anchors.bottom: parent.bottom
    anchors.right: rightEdge() ? parent.right : undefined
    anchors.left: rightEdge() ? undefined : parent.left
    width: 24 + sliderContainerLeft.width
    opacity: 1

    Rectangle {
        id: sliderContainerLeft
        height: parent.height
        width: 160//sliderTogglerLeft.checked ? defheigth + 120 : defheigth
        color: lightgrey
        Material.accent: mainColor
        Material.foreground: orange

        property string labelBorderColor: "transparent"

        Column {
            spacing: 10
            id: sliderRow
            width: defheigth

            Column {
                id: main_btns
                width: 160

                Button {
                    id: addTracks
                    highlighted: true
                    width: parent.width
                    height: defheigth

                    text: qsTr("Add Path")

                    /*onHoveredChanged: function () {
                        addTracksRect.color = (addTracksRect.color === blue) ? orange : blue
                    }*/

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
                    height: defheigth
                    visible: columnTrack.children.length > 0
                    enabled: true
                    text: "Delete Paths"
                    width: parent.width
                    //highlighted: true

                    /*onHoveredChanged: function () {
                        trackBG.color = (trackBG.color === blue) ? orange : blue
                    }*/

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

            Column {
                id: confirm_btns
                width: defheigth + 120
                visible: false
                Button {
                    id: abort
                    text:"No"
                    width: parent.width
                    height: defheigth
                    highlighted: true

                    onClicked: function () {
                        restoreBtns()
                        deselect_all()
                        enableBtns(y)
                        bar_manage.inhibit = false
                        bar_manage.hide_all()
                        bar_manage.show_manage()

                    }

                    /*onHoveredChanged: function () {
                        closeimg.color = (closeimg.color === red) ? lightred : red
                    }*/

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
                    text: "Yes"
                    enabled: true
                    height: defheigth
                    highlighted: true
                    width: parent.width

                    onClicked: function(){
                        delete_toggled_items()
                        bar_manage.inhibit = false
                    }

                    /*onHoveredChanged: function () {
                        checkimg.color = (checkimg.color === green) ? lightgreen : green
                    }*/

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
                id: columnTrack
                width: defheigth + 120
                y: 96
                //property bool expanded: sliderTogglerLeft.checked
            }
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
        for (var i = 0; i < columnTrack.children.length; i++) {
            columnTrack.children[i].managed_path.check_safe(map.polysec_cur)
        }
    }

    function deselect_all() {
        for (var i = 0; i < columnTrack.children.length; i++) {
            var c = columnTrack.children[i]
            c.highlight(false)
        }
    }
    function update_selection(poly) {
        if (!multichoice) {
            for (var i = 0; i < columnTrack.children.length; i++) {
                var c = columnTrack.children[i]
                c.highlight(poly === c.managed_path)
            }
        } else {
            for (i = 0; i < columnTrack.children.length; i++) {
                c = columnTrack.children[i]
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
        var c = columnTrack.children[idx]
        delete_item(c)
    }

    function delete_all() {
        for (var i = columnTrack.children.length - 1; i >= 0; i--)
            delete_item_at_idx(i)
    }

    function delete_toggled_items() {
        bar_manage.hide_all()
        for (var i = columnTrack.children.length - 1; i >= 0; i--) {
            var c = columnTrack.children[i]
            if (c.toggled) {
                delete_item_at_idx(i)
            }
        }
        restoreBtns()
        deselect_all()
    }
}
