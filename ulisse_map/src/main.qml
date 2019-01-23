import QtQuick 2.0
import QtQuick.Window 2.0
import QtQuick.Layouts 1.3
import QtLocation 5.6
import QtPositioning 5.6

Window {
    width: 800
    height: 600
    visible: true

    minimumHeight: 200
    minimumWidth: 300

    Plugin {
        id: mapPlugin
        name: "osm" // "mapboxgl", "esri", ...
        // specify plugin parameters if necessary
        // PluginParameter {
        //     name:
        //     value:
        // }
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
                    Layout.preferredHeight: 40
                    color: 'white'
                    border.width: 4
                    border.color: 'lightgray'

                    Text {
                        id: textlatlong
                        opacity: 0.0
                        anchors.centerIn: parent
                        text: "LatLong: %1, %2".arg(marker.coordinate.latitude).arg(marker.coordinate.longitude)
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
                    //Layout.preferredWidth: 70
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
                    id:marker
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
                             marker.opacity = 1.0
                             textlatlong.opacity = 1.0
                             marker.coordinate = map.toCoordinate(Qt.point(mouse.x,mouse.y))
                         }
                    }
                }
            }
        }
    }
}
