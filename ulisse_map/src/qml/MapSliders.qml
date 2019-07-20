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
import QtQuick.Controls.Material 2.1

Row {
    id: containerRow

    property var mapSource
    property real fontSize: 14
    property color labelBackground: "transparent"
    property int edge: Qt.RightEdge
    property var togglerColor: mainAccentColor

    function rightEdge() {
        return (containerRow.edge === Qt.RightEdge)
    }

    layoutDirection: rightEdge() ? Qt.LeftToRight : Qt.RightToLeft
    anchors.top: parent.top
    anchors.bottom: parent.bottom
    anchors.right: rightEdge() ? parent.right : undefined
    anchors.left: rightEdge() ? undefined : parent.left
    width: sliderToggler.width + (sliderToggler.checked ? sliderContainer.width : 0)

    C1.Button {
        id: sliderToggler
        width: 24
        height: 72
        checkable: true
        checked: false
        anchors.verticalCenter: parent.verticalCenter

        transform: Scale {
            origin.x: rightEdge() ? 0 : sliderToggler.width / 2
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
            width: sliderToggler.width / 2
            height: sliderToggler.height / 2
            color: togglerColor
            antialiasing: true
            opacity: sliderToggler.buttonOpacity
            anchors.top: parent.top
            anchors.left: sliderToggler.checked ? parent.left : parent.horizontalCenter
            anchors.leftMargin: -5

            transform: Matrix4x4 {
                property real d: sliderToggler.checked ? 1.0 : -1.0
                matrix: Qt.matrix4x4(1.0, d * sliderToggler.shear, 0.0,
                                     0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                                     0.0, 0.0, 0.0, 0.0, 1.0)
            }
        }

        Rectangle {
            width: sliderToggler.width / 2
            height: sliderToggler.height / 2
            color: togglerColor
            antialiasing: true
            opacity: sliderToggler.buttonOpacity
            anchors.top: parent.verticalCenter
            anchors.right: sliderToggler.checked ? parent.right : parent.horizontalCenter
            anchors.rightMargin: 5
            transform: Matrix4x4 {
                property real d: sliderToggler.checked ? -1.0 : 1.0
                matrix: Qt.matrix4x4(1.0, d * sliderToggler.shear, 0.0,
                                     0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                                     0.0, 0.0, 0.0, 0.0, 1.0)
            }
        }
    }

    Rectangle {
        id: sliderContainer
        height: parent.height
        width: sliderRow.width + 10
        visible: sliderToggler.checked
        color: Qt.rgba(0, 0, 0,
                       0.05) //Qt.rgba( 0, 191 / 255.0, 255 / 255.0, 0.1)

        Material.accent: mainAccentColor
        Material.foreground: Material.color(Material.BlueGrey,
                                            Material.Shade600)

        property string labelBorderColor: "transparent"
        property real slidersHeight: sliderContainer.height - rowSliderValues.height
                                     - rowSliderLabels.height - sliderColumn.spacing * 2
                                     - sliderColumn.topPadding - sliderColumn.bottomPadding

        Column {
            id: sliderColumn
            spacing: 10
            topPadding: 16
            bottomPadding: 48
            anchors.centerIn: parent

            // the sliders value labels
            Row {
                id: rowSliderValues
                spacing: sliderRow.spacing
                width: sliderRow.width
                height: 32
                property real entryWidth: zoomSlider.width

                Rectangle {
                    color: labelBackground
                    height: parent.height
                    width: parent.entryWidth
                    border.color: sliderContainer.labelBorderColor
                    Label {
                        id: labelZoomValue
                        text: zoomSlider.value.toFixed(3)
                        font.pixelSize: fontSize
                        rotation: -90
                        anchors.centerIn: parent
                    }
                }
                Rectangle {
                    color: labelBackground
                    height: parent.height
                    width: parent.entryWidth
                    border.color: sliderContainer.labelBorderColor
                    Label {
                        id: labelBearingValue
                        text: bearingSlider.value.toFixed(2)
                        font.pixelSize: fontSize
                        rotation: -90
                        anchors.centerIn: parent
                    }
                }
                Rectangle {
                    color: labelBackground
                    height: parent.height
                    width: parent.entryWidth
                    border.color: sliderContainer.labelBorderColor
                    Label {
                        id: labelTiltValue
                        text: tiltSlider.value.toFixed(2)
                        font.pixelSize: fontSize
                        rotation: -90
                        anchors.centerIn: parent
                    }
                }
                Rectangle {
                    color: labelBackground
                    height: parent.height
                    width: parent.entryWidth
                    border.color: sliderContainer.labelBorderColor
                    Label {
                        id: labelFovValue
                        text: fovSlider.value.toFixed(2)
                        font.pixelSize: fontSize
                        rotation: -90
                        anchors.centerIn: parent
                    }
                }
            } // rowSliderValues

            // The sliders row
            Row {
                spacing: -10
                id: sliderRow
                height: sliderContainer.slidersHeight

                Slider {
                    id: zoomSlider
                    height: parent.height
                    orientation: Qt.Vertical
                    from: containerRow.mapSource.minimumZoomLevel
                    to: containerRow.mapSource.maximumZoomLevel
                    value: containerRow.mapSource.zoomLevel
                    onValueChanged: {
                        containerRow.mapSource.zoomLevel = value
                    }
                }
                Slider {
                    id: bearingSlider
                    height: parent.height
                    from: 0
                    to: 360
                    orientation: Qt.Vertical
                    value: containerRow.mapSource.bearing
                    onValueChanged: {
                        containerRow.mapSource.bearing = value
                    }
                }
                Slider {
                    id: tiltSlider
                    height: parent.height
                    orientation: Qt.Vertical
                    from: containerRow.mapSource.minimumTilt
                    to: containerRow.mapSource.maximumTilt
                    value: containerRow.mapSource.tilt
                    onValueChanged: {
                        containerRow.mapSource.tilt = value
                    }
                }
                Slider {
                    id: fovSlider
                    height: parent.height
                    orientation: Qt.Vertical
                    from: containerRow.mapSource.minimumFieldOfView
                    to: containerRow.mapSource.maximumFieldOfView
                    value: containerRow.mapSource.fieldOfView
                    onValueChanged: {
                        containerRow.mapSource.fieldOfView = value
                    }
                }
            } // Row sliders

            // The labels row
            Row {
                id: rowSliderLabels
                spacing: sliderRow.spacing
                width: sliderRow.width
                property real entryWidth: zoomSlider.width
                property real entryHeight: 64

                Rectangle {
                    color: labelBackground
                    height: parent.entryHeight
                    width: parent.entryWidth
                    border.color: sliderContainer.labelBorderColor
                    Label {
                        id: labelZoom
                        text: "Zoom"
                        font.pixelSize: fontSize
                        rotation: -90
                        anchors.centerIn: parent
                    }
                }

                Rectangle {
                    color: labelBackground
                    height: parent.entryHeight
                    width: parent.entryWidth
                    border.color: sliderContainer.labelBorderColor
                    Label {
                        id: labelBearing
                        text: "Bearing"
                        font.pixelSize: fontSize
                        rotation: -90
                        anchors.centerIn: parent
                    }
                }
                Rectangle {
                    color: labelBackground
                    height: parent.entryHeight
                    width: parent.entryWidth
                    border.color: sliderContainer.labelBorderColor
                    Label {
                        id: labelTilt
                        text: "Tilt"
                        font.pixelSize: fontSize
                        rotation: -90
                        anchors.centerIn: parent
                    }
                }
                Rectangle {
                    color: labelBackground
                    height: parent.entryHeight
                    width: parent.entryWidth
                    border.color: sliderContainer.labelBorderColor
                    Label {
                        id: labelFov
                        text: "FoV"
                        font.pixelSize: fontSize
                        rotation: -90
                        anchors.centerIn: parent
                    }
                }
            } // rowSliderLabels
        } // Column
    }
    states: [
        State {
            name: "opened"
            PropertyChanges {
                target: sliderToggler
                checked: true
            }
        },
        State {
            name: "closed"
            PropertyChanges {
                target: sliderToggler
                checked: false
            }
        }
    ]
    // sliderContainer
} // containerRow
