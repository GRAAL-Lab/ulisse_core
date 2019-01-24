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

                    Column {
                        id: markerColumn
                        height: textTitleMarker.contentHeight + textMarker.contentHeight
                        width: parent.width
                        spacing: 2

                        Label {
                            id: textTitleMarker
                            color: 'lightgray'
                            font.pointSize: 9
                            text: "Marker Coordinates"
                        }

                        Label {
                            id: textMarker
                            color: 'lightgray'
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

                    Column {
                        id: ulisseTextColumn
                        height: textTitleUlissePos.contentHeight + textUlissePos.contentHeight
                        width: parent.width

                        Label {
                            id: textTitleUlissePos
                            color: 'lightgray'
                            font.pointSize: 9
                            text: "Ulisse Coordinates"
                        }

                        Label {
                            id: textUlissePos
                            color: 'lightgray'
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
                center: QtPositioning.coordinate(44.4, 8.94) // Genoa
                zoomLevel: 14

                MapQuickItem {
                    id:markerIcon
                    sourceItem: Image{
                        id: image
                        width: 32; height: 32
                        source: 'images/map-marker-64.png'

                    }
                    coordinate: map.center
                    anchorPoint.x: image.width / 2
                    anchorPoint.y: image.height / 2
                    opacity: 0.0
                }

                MouseArea {
                    anchors.fill: parent
                    acceptedButtons: Qt.LeftButton | Qt.RightButton
                    onClicked: {
                         if(mouse.button & Qt.RightButton) {
                             marker_coords = map.toCoordinate(Qt.point(mouse.x,mouse.y))

                             markerIcon.opacity = 1.0
                             markerIcon.coordinate = map.toCoordinate(Qt.point(mouse.x,mouse.y-16))

                             textMarker.text = "LatLong: %1, %2".arg(marker_coords.latitude).arg(marker_coords.longitude)
                             textMarker.color = 'black'

                         }
                    }
                }
            }
        }
    }
}
