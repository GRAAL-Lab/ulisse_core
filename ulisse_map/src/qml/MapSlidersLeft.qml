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
    property alias expanded: sliderTogglerLeft.checked
    property var togglerColor: mainAccentColor
    property alias columnTrack: columnTrack
    property alias sliderW: sliderTogglerLeft.width

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
                // height: sliderContainerLeft.slidersHeight
                width: 30
                Column{
                    id:buttons3
                    Button{
                        id:addTracks
                        text:"+"
                        width: sliderTogglerLeft.checked ? sliderRow.width + 120 : sliderRow.width
                        onClicked: function(){
                            pathRectPoly.show_shape_choice()
                        }
                        background: Rectangle {
                            id: addTracksRect
                            color: "#abcdef"
                        }
                    }
                    Button{
                        id:saveTracks
                        onClicked: function() {
                            if(mapView.currentState === mapView.generalState.deletemode){
                             mapView.currentState = mapView.generalState.empty
                            }
                            return
                        }
                        text:"s"
                        enabled: true
                        width:sliderTogglerLeft.checked ? sliderRow.width + 120 : sliderRow.width
                        background: Rectangle {
                        id: saveTracksRect
                        color: "#abcdef"
                        }
                    }
                    Button{
                        id:deleteTracks
                        text:"x"
                        enabled: true
                        width:sliderTogglerLeft.checked ? sliderRow.width + 120 : sliderRow.width
                        onClicked: mapView.currentState = mapView.generalState.deletemode
                        background: Rectangle {
                        color: "#abcdef"
                        }
                    }
                }

                Column{
                    id:columnTrack
                }
            }
    }
    states: [
        State {
            name: "opened"
            PropertyChanges {
                target: sliderTogglerLeft
                checked: true
            }
        },
        State {
            name: "closed"
            PropertyChanges {
                target: sliderTogglerLeft
                checked: false
            }
        },
        State   {
            name: "deletemode"
            PropertyChanges {
                target: deleteTracks
                text: sliderTogglerLeft.checked ?  "Confirm?" : "?"
            }
            PropertyChanges {
                target: addTracks
                text: sliderTogglerLeft.checked ? "Yes" :"y"
            }
            PropertyChanges {
                target: addTracksRect
                color: "#00ff00"
            }
            PropertyChanges {
                target: saveTracks
                text:sliderTogglerLeft.checked ?  "No" : "n"
            }
            PropertyChanges {
                target: saveTracksRect
                color: "#ff0000"
            }
        }

    ]
    // sliderContainerLeft
} // containerRowLeft
