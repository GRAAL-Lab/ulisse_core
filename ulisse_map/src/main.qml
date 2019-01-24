import QtQuick 2.1
import QtQuick.Window 2.0
import QtQuick.Layouts 1.3
import QtQuick.Controls 1.2
import QtLocation 5.6
import QtPositioning 5.6

Window {
    width: 800
    height: 600
    visible: true

    minimumHeight: 200
    minimumWidth: 300

    property var marker_coords: QtPositioning.coordinate(44.4, 8.94)

    Plugin {
        id: mapPlugin
        name: "osm" // "mapboxgl", "esri", ...
        // specify plugin parameters if necessary
         PluginParameter {
             name: "osm.mapping.offline.directory"
             value: "/home/graal/.cache/QtLocation/5.8/tiles/osm/"
         }
    }

    RowLayout {
        id: layout
        anchors.fill: parent
        spacing: 6

        Rectangle {
            id: leftbar
            color: 'lightgray'
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.minimumWidth: 200
            Layout.minimumHeight: 150
            Layout.preferredWidth: 100
            Layout.maximumWidth: 250

            ColumnLayout{
                anchors.fill: parent
                spacing: 2

                Rectangle {
                    Layout.alignment: Qt.AlignCenter
                    Layout.fillWidth: true
                    Layout.preferredHeight: markerColumn.height
                    color: 'white'
                    border.width: 2
                    border.color: 'lightgray'

                    ColumnLayout {
                        id: markerColumn
                        height: textTitleMarker.contentHeight + textMarker.contentHeight + 5
                        width: parent.width
                        spacing: 2

                        Label {
                            id: textTitleMarker
                            Layout.alignment: Qt.AlignCenter
                            Layout.fillWidth: true
                            color: 'gray'
                            font.pointSize: 9
                            text: "Marker Coordinates"
                        }

                        Label {
                            id: textMarker
                            Layout.alignment: Qt.AlignCenter
                            Layout.fillWidth: true
                            color: 'lightgray'
                            font.pointSize: 11
                            text: "Right click on map"
                        }
                    }
                }

                Rectangle {
                    Layout.alignment: Qt.AlignCenter
                    Layout.fillWidth: true
                    Layout.preferredHeight: 40
                    color: 'white'
                    border.width: 2
                    border.color: 'lightgray'

                    ColumnLayout {
                        id: ulisseTextColumn
                        height: textTitleUlissePos.contentHeight + textUlissePos.contentHeight + 5
                        width: parent.width

                        Label {
                            id: textTitleUlissePos
                            Layout.alignment: Qt.AlignCenter
                            Layout.fillWidth: true
                            color: 'gray'
                            font.pointSize: 9
                            text: "Ulisse Coordinates"
                        }

                        Label {
                            id: textUlissePos
                            Layout.alignment: Qt.AlignCenter
                            Layout.fillWidth: true
                            font.pointSize: 11
                            text: "%1, %2".arg(fbkUpdater.ulisse_pos.latitude).arg(fbkUpdater.ulisse_pos.longitude)
                        }
                    }
                }

                Rectangle {
                    Layout.alignment: Qt.AlignCenter
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    color: 'transparent'

                }

                Rectangle {
                    Layout.alignment: Qt.AlignBottom
                    Layout.fillWidth: true
                    Layout.preferredHeight: 40
                    color: 'transparent'
                    Text {
                        anchors.centerIn: parent
                        text: "(Right click to set marker)"
                    }
                }
            }
        }

        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.minimumWidth: 100
            Layout.preferredWidth: 200
            Layout.preferredHeight: 100

            Map {
                id: map
                anchors.fill: parent
                plugin: mapPlugin
                center: QtPositioning.coordinate(44.393, 8.945) // Genoa
                zoomLevel: 14

                MapQuickItem {
                    id:markerIcon
                    sourceItem: Image{
                        id: markerImage
                        width: 32; height: 32
                        source: 'images/map-marker-64.png'

                    }
                    coordinate: map.center
                    anchorPoint.x: markerImage.width / 2
                    anchorPoint.y: markerImage.height / 2
                    opacity: 0.0
                }

                MapQuickItem {
                    id: ulisseIcon
                    sourceItem: Image{
                        id: ulisseImage
                        width: 32; height: 32
                        source: 'images/catamaran_icon_32.png'
                        transform: Rotation { origin.x: 16; origin.y: 16; angle: fbkUpdater.ulisse_yaw_deg}
                    }
                    coordinate: QtPositioning.coordinate(fbkUpdater.ulisse_pos.latitude, fbkUpdater.ulisse_pos.longitude) // Genoa
                    anchorPoint.x: ulisseImage.width / 2
                    anchorPoint.y: ulisseImage.height / 2
                }

                MouseArea {
                    anchors.fill: parent
                    acceptedButtons: Qt.LeftButton | Qt.RightButton
                    onClicked: {
                         if(mouse.button & Qt.RightButton) {
                             marker_coords = map.toCoordinate(Qt.point(mouse.x,mouse.y))

                             markerIcon.opacity = 1.0
                             markerIcon.coordinate = map.toCoordinate(Qt.point(mouse.x,mouse.y-16))

                             textMarker.text = "%1, %2".arg(marker_coords.latitude).arg(marker_coords.longitude)
                             textMarker.color = 'black'
                         }
                    }
                }
            }
        }
    }
}
