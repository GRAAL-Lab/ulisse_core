import QtQuick 2.6
import QtQuick.Controls 1.4 as C1
import QtQuick.Controls.Styles 1.4 as C1S
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import "."

Row {
    id: containerRow

    property var mapSource
    property real fontSize: 14
    property color labelBackground: "transparent"
    property int edge: Qt.RightEdge
    property color togglerColor: orange

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
        y: parent.y + 350

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
        Material.accent: orange
        Material.foreground: grey

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
                    border.color: labelBackground
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
                    border.color: labelBackground
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
                    border.color: labelBackground
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
                    border.color: labelBackground
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
                    border.color: labelBackground
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
                    border.color: labelBackground
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
                    border.color: labelBackground
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
                    border.color: labelBackground
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
