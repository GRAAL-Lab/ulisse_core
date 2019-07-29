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
    property var defheigth: 41
    property color labelBackground: "transparent"
    property int edge: Qt.LeftEdge
    property var togglerColor: orange
    property alias sliderW: sliderTogglerLeft.width
    property bool multichoice: false

    state: {
        0: "empty",
                1: "path",
                2: "rect",
                3: "poly",
                4: "polysec",
                5: "editmode",
                6: "deletemode"
    }[mapView.currentState]

    function rightEdge() {
        return (containerRowLeft.edge === Qt.RightEdge)
    }

        layoutDirection: rightEdge() ? Qt.LeftToRight : Qt.RightToLeft
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        anchors.right: rightEdge() ? parent.right : undefined
        anchors.left: rightEdge() ? undefined : parent.left
        width: sliderTogglerLeft.width + sliderContainerLeft.width
        opacity: 1

        C1.Button {
            id: sliderTogglerLeft
            width: 24
            height: 72
            checkable: true
            checked: false
            y: parent.y + 350

            transform: Scale {
                origin.x: rightEdge() ? 0 : sliderTogglerLeft.width / 2
                xScale: rightEdge() ? 1 : -1
            }

            style: C1S.ButtonStyle {
                background: Rectangle {
                    color: "transparent"
                }
            }

            property real shear: 0.333
            property real mirror: rightEdge() ? 1.0 : -1.0

            Rectangle {
                width: sliderTogglerLeft.width / 2
                height: sliderTogglerLeft.height / 2
                color: togglerColor
                antialiasing: true
                anchors.top: parent.top
                anchors.left: sliderTogglerLeft.checked ? parent.left : parent.horizontalCenter
                anchors.leftMargin: -5

                transform: Matrix4x4 {
                    property real d: sliderTogglerLeft.checked ? 1.0 : -1.0
                    matrix: Qt.matrix4x4(1.0, d * sliderTogglerLeft.shear, 0.0,
                                         0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                         0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
                }
            }

            Rectangle {
                width: sliderTogglerLeft.width / 2
                height: sliderTogglerLeft.height / 2
                color: togglerColor
                antialiasing: true
                anchors.top: parent.verticalCenter
                anchors.right: sliderTogglerLeft.checked ? parent.right : parent.horizontalCenter
                anchors.rightMargin: 5
                transform: Matrix4x4 {
                    property real d: sliderTogglerLeft.checked ? -1.0 : 1.0
                    matrix: Qt.matrix4x4(1.0, d * sliderTogglerLeft.shear, 0.0,
                                         0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                         0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
                }
            }
        }

        Rectangle {
            id: sliderContainerLeft
            height: parent.height
            width: sliderTogglerLeft.checked ? defheigth + 120 : defheigth
            color: lightgrey
            Material.accent: blue
            Material.foreground: orange

            property string labelBorderColor: "transparent"

            Column {
                spacing: 10
                id: sliderRow
                width: defheigth

                Column {
                    id: main_btns
                    width: sliderTogglerLeft.checked ? defheigth + 120 : defheigth

                    Button {
                        id: addTracks
                        highlighted: true
                        width: parent.width
                        height: defheigth

                        text: (sliderTogglerLeft.checked) ? qsTr("Add Path") : qsTr(
                                                                "")

                        onHoveredChanged: function () {
                            addTracksRect.color = (addTracksRect.color === blue) ? orange : blue
                        }

                        onClicked: function () {
                            bar_manage.show_shape_choice()
                            enableBtns(false)
                        }

                        background: Rectangle {
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
                        }
                    }

                    Button {
                        id: deleteTracks
                        height: defheigth
                        visible: columnTrack.children.length > 0
                        enabled: true
                        text: sliderTogglerLeft.checked ? "Delete Paths" : ""
                        width: parent.width
                        highlighted: true

                        onHoveredChanged: function () {
                            trackBG.color = (trackBG.color === blue) ? orange : blue
                        }

                        onClicked: function () {
                            multichoice = true
                            bar_manage.inhibit = true
                            bar_manage.hide_all()
                            sidebar_manage.deselect_all()
                            main_btns.visible = false
                            confirm_btns.visible = true
                        }

                        background: Rectangle {
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
                        }
                    }
                }

                Column {
                    id: confirm_btns
                    width: sliderTogglerLeft.checked ? defheigth + 120 : defheigth
                    visible: false
                    Button {
                        id: abort
                        text: sliderTogglerLeft.checked ? "No" : ""
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

                        onHoveredChanged: function () {
                            closeimg.color = (closeimg.color === red) ? lightred : red
                        }

                        background: Rectangle {
                            id: closeimg
                            Image {
                                visible: sliderTogglerLeft.checked ? false : true
                                anchors.horizontalCenter: parent.horizontalCenter
                                anchors.verticalCenter: parent.verticalCenter
                                source: 'qrc:/images/close.png'
                            }
                            color: red
                        }
                    }
                    Button {
                        id: confirm
                        text: sliderTogglerLeft.checked ? "Yes" : ""
                        enabled: true
                        height: defheigth
                        highlighted: true
                        width: parent.width

                        onClicked: function(){
                            delete_items()
                            bar_manage.inhibit = false
                        }

                        onHoveredChanged: function () {
                            checkimg.color = (checkimg.color === green) ? lightgreen : green
                        }

                        background: Rectangle {
                            color: green
                            id: checkimg
                            Image {
                                visible: sliderTogglerLeft.checked ? false : true
                                anchors.horizontalCenter: parent.horizontalCenter
                                anchors.verticalCenter: parent.verticalCenter
                                source: 'qrc:/images/check.png'
                            }
                        }
                    }
                }

                Column {
                    id: columnTrack
                    width: sliderTogglerLeft.checked ? defheigth + 120 : defheigth
                    y: 96
                    property bool expanded: sliderTogglerLeft.checked
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

        function delete_all() {
            for (var i = columnTrack.children.length - 1; i >= 0; i--) {
                var c = columnTrack.children[i]
                c.managed_path.deregister_map_items()
                map.removeMapItem(c.managed_path)
                c.destroy()
                bar_manage.n--
            }
        }

        function delete_items() {
            bar_manage.hide_all()
            for (var i = columnTrack.children.length - 1; i >= 0; i--) {
                var c = columnTrack.children[i]
                if (c.toggled) {
                    c.managed_path.deregister_map_items()
                    map.removeMapItem(c.managed_path)
                    c.destroy()
                    bar_manage.n--
                }
            }
            restoreBtns()
            deselect_all()
        }
    }
