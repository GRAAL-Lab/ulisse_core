/****************************************************************************
**
** Copyright (C) 2017 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licenses holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/
import QtQuick 2.6
import QtQuick.Controls 1.4 as C1
import QtQuick.Controls.Styles 1.4 as C1S
import QtQuick.Controls 2.2
import QtQml.Models 2.1
import QtQuick.Controls.Material 2.1


Row {
    id: containerRowLeft

    property var mapSource
    property alias columnTrack: columnTrack
    property real fontSize: 14
    property color labelBackground: "transparent"
    property int edge: Qt.LeftEdge
    property var togglerColor: mainAccentColor
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

    C1.Button {
        id: sliderTogglerLeft
        width: 24
        height: 72
        checkable: true
        checked: false
        anchors.verticalCenter: parent.verticalCenter

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
        property real buttonOpacity: 0.66
        property real mirror: rightEdge() ? 1.0 : -1.0

        Rectangle {
            width: sliderTogglerLeft.width / 2
            height: sliderTogglerLeft.height / 2
            color: togglerColor
            antialiasing: true
            opacity: sliderTogglerLeft.buttonOpacity
            anchors.top: parent.top
            anchors.left: sliderTogglerLeft.checked ? parent.left : parent.horizontalCenter
            anchors.leftMargin: -5

            transform: Matrix4x4 {
                property real d: sliderTogglerLeft.checked ? 1.0 : -1.0
                matrix: Qt.matrix4x4(1.0, d * sliderTogglerLeft.shear, 0.0,
                                     0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                                     0.0, 0.0, 0.0, 0.0, 1.0)
            }
        }

        Rectangle {
            width: sliderTogglerLeft.width / 2
            height: sliderTogglerLeft.height / 2
            color: togglerColor
            antialiasing: true
            opacity: sliderTogglerLeft.buttonOpacity
            anchors.top: parent.verticalCenter
            anchors.right: sliderTogglerLeft.checked ? parent.right : parent.horizontalCenter
            anchors.rightMargin: 5
            transform: Matrix4x4 {
                property real d: sliderTogglerLeft.checked ? -1.0 : 1.0
                matrix: Qt.matrix4x4(1.0, d * sliderTogglerLeft.shear, 0.0,
                                     0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                                     0.0, 0.0, 0.0, 0.0, 1.0)
            }
        }
    }

    Rectangle {
        id: sliderContainerLeft
        height: parent.height
        width: sliderTogglerLeft.checked ? sliderRow.width + 120 : sliderRow.width
        color: Qt.rgba(0, 0, 0,
                       0.05) //Qt.rgba( 0, 191 / 255.0, 255 / 255.0, 0.1)

        Material.accent: mainAccentColor
        Material.foreground: Material.color(Material.BlueGrey,
                                            Material.Shade600)

        property string labelBorderColor: "transparent"

        Column {
            spacing: 10
            id: sliderRow
            width: 30

            Column{
                id: main_btns
                width: sliderTogglerLeft.checked ? sliderRow.width + 120 : sliderRow.width
                Button{
                    id:addTracks
                    text:"+"
                    width: parent.width
                    onClicked: function(){
                        pathRectPoly.show_shape_choice()
                        enableBtns(false)
                    }
                    background: Rectangle {
                        id: addTracksRect
                        color: "#abcdef"
                    }
                }
                Button{
                    id:saveTracks
                    onClicked: function() {
                        multichoice = true
                        main_btns.visible = false
                        confirm_btns.visible = true
                        confirm.clicked.disconnect(save_items)
                        confirm.clicked.disconnect(delete_items)
                        confirm.clicked.connect(save_items)
                    }
                    text:"s"
                    enabled: true
                    width: parent.width
                    background: Rectangle {
                    id: saveTracksRect
                    color: "#abcdef"
                    }
                }
                Button{
                    id:deleteTracks
                    text:"x"
                    visible: (pathRectPoly.n>0)? true : false
                    enabled: true
                    width: parent.width
                    onClicked: function(){
                        multichoice = true
                        main_btns.visible = false
                        confirm_btns.visible = true
                        pathRectPoly.enableBtns(false)
                        confirm.clicked.disconnect(save_items)
                        confirm.clicked.disconnect(delete_items)
                        confirm.clicked.connect(delete_items)
                    }
                    background: Rectangle {
                    color: "#abcdef"
                    }
                }
            }

            Column{
                id: confirm_btns
                width: sliderTogglerLeft.checked ? sliderRow.width + 120 : sliderRow.width
                visible: false
                Button{
                    id:abort
                    text:"n"
                    width: parent.width
                    onClicked: function(){
                        restoreBtns()
                        deselect_all()
                        enableBtns(y)
                        pathRectPoly.hide_all()
                        pathRectPoly.enableBtns(true)
                    }
                    background: Rectangle {
                    color: "#ff0000"
                    }
                }
                Button{
                    id:confirm
                    text:"y"
                    enabled: true
                    width: parent.width
                    onClicked: function(){
                        pathRectPoly.enableBtns(true)
                        enableBtns(y)}
                    background: Rectangle {
                    color: "#00ff00"
                    }
                }
            }

            Column {
                width: sliderTogglerLeft.checked ? sliderRow.width + 120 : sliderRow.width
                property bool expanded: sliderTogglerLeft.checked
                y: 96
                id:columnTrack
            }
        }
    }

    function enableBtns(y){
        addTracks.enabled = y
        saveTracks.enabled = y
        deleteTracks.enabled = y
    }

    function restoreBtns(){
        main_btns.visible = true
        confirm_btns.visible = false
        enableBtns(true)
        multichoice = false
    }

    function deselect_all(){
        for (var i = 0; i<columnTrack.children.length; i++){
            var c = columnTrack.children[i]
            c.highlight(false)
        }
    }
    function update_selection(poly){
        if (!multichoice){
            for (var i = 0; i<columnTrack.children.length; i++){
                var c = columnTrack.children[i]
                c.highlight(poly === c._comp)
            }
        } else {
            for (i = 0; i<columnTrack.children.length; i++){
                c = columnTrack.children[i]
                if (poly === c._comp) c.toggle()
            }
        }
    }

    function save_items(){
        //TODO
        restoreBtns()
        deselect_all()
        pathRectPoly.hide_all()
    }

    function delete_items(){
        pathRectPoly.hide_all()
        for (var i = columnTrack.children.length-1; i>=0; i--){
            var c = columnTrack.children[i]
            if (c.toggled){
                c._comp.deregister_map_items()
                map.removeMapItem(c._comp)
                c.destroy()
                pathRectPoly.n--
                console.log(pathRectPoly.n)
            }
        }
        restoreBtns()
        deselect_all()
    }
}
