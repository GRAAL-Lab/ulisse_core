import QtQuick 2.6
import QtQuick.Window 2.0
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtLocation 5.6
import QtPositioning 5.6
import Qt.labs.settings 1.0
import QtQuick.Controls.Material 2.1
import QtQuick.Controls.Universal 2.1
import QtQuick.Controls.Styles 1.4
import "./qml"

Window {
    width: 800
    height: 600
    visible: true

    minimumHeight: 200
    minimumWidth: 300

    /* Stopping Arms when space is pressed */
    Shortcut {
        sequence: " "
        //onActivated: commanderView.stopArms()
    }

    //Material.theme: Material.Dark
    Settings {
        id: settings
        property string style: "Material"
    }


    property var marker_coords: QtPositioning.coordinate(44.4, 8.94)

    Plugin {
        id: mapPlugin
        name: "osm"
         PluginParameter {
             name: "osm.mapping.offline.directory"
             value: "/home/graal/.cache/QtLocation/5.8/tiles/osm/"
         }
    }

    RowLayout {
        id: mainlayout
        anchors.fill: parent
        spacing: 4

        Rectangle {
            id: leftbarrect
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.minimumWidth: 200
            Layout.minimumHeight: 150
            Layout.preferredWidth: 100
            Layout.maximumWidth: 250

            ColumnLayout {
                id: leftbarlayout
                anchors.fill: parent
                spacing: 0

                Rectangle {
                    id: infodatarect
                    Layout.alignment: Qt.AlignCenter
                    Layout.fillWidth: true
                    //color: 'white'
                    Layout.preferredHeight: infodatalayout.height

                    ColumnLayout {
                        id: infodatalayout
                        width: parent.width
                        Layout.preferredHeight: markerTextLabel.height + ulisseTextLabel.height + 20
                        spacing: 10

                        LabelledText {
                            id: markerTextLabel
                            label: "Marker Coordinates"
                            text: "Right click on map"
                            textColor: 'lightgray'
                        }

                        LabelledText {
                            id: ulisseTextLabel
                            label: "Ulisse Coordinates"
                            text: "%1, %2".arg(fbkUpdater.ulisse_pos.latitude).arg(fbkUpdater.ulisse_pos.longitude)
                            textColor: 'darkslategray'
                        }

                        LabelledText {
                            id: goalTextLabel
                            label: "Goal Coordinates"
                            text: "%1, %2".arg(fbkUpdater.goal_pos.latitude).arg(fbkUpdater.goal_pos.longitude)
                            textColor: 'darkslategray'
                        }
                    }
                }

                Rectangle {
                    id: commandRect
                    Layout.alignment: Qt.AlignCenter
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    color: 'lightgray'

                    ColumnLayout {
                        id: buttonsColumn
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                        Label {
                            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                            font.pointSize: 12
                            font.weight: Font.DemiBold
                            bottomPadding: 10
                            color: 'seagreen'
                            text: "Commands"

                        }

                        Button {
                            text: "Halt"
                            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                            //onClicked: model.submit()
                        }
                        Button {
                            text: "Hold Position"
                            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                            //onClicked: model.revert()
                        }
                        Button {
                            text: "Move To Marker"
                            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                            //onClicked: model.submit()
                        }
                        Button {
                            text: "Speed-Heading"
                            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                            //onClicked: model.revert()
                        }
                    }

                }

                Rectangle {
                    Layout.alignment: Qt.AlignBottom
                    Layout.fillWidth: true
                    Layout.preferredHeight: 40
                    color: 'lightgray'
                    Text {
                        anchors.centerIn: parent
                        color: 'darkslategray'
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
                    anchorPoint.y: markerImage.height
                    opacity: 0.0
                }

                MapQuickItem {
                    id: ulisseIcon
                    sourceItem: Image{
                        id: ulisseImage
                        width: 38; height: 38
                        source: 'images/catamaran_icon_64.png'
                        transform: Rotation { origin.x: 16; origin.y: 16; angle: fbkUpdater.ulisse_yaw_deg }
                    }
                    coordinate: QtPositioning.coordinate(fbkUpdater.ulisse_pos.latitude, fbkUpdater.ulisse_pos.longitude)
                    anchorPoint.x: ulisseImage.width / 2
                    anchorPoint.y: ulisseImage.height / 2
                }

                MapQuickItem {
                    id: goalFlag
                    objectName: "goalFlag"
                    sourceItem: Image{
                        id: flagCheckerImage
                        width: 38; height: 38
                        source: 'images/flag_checker.png'
                    }
                    coordinate: QtPositioning.coordinate(fbkUpdater.goal_pos.latitude, fbkUpdater.goal_pos.longitude)
                    anchorPoint.x: flagCheckerImage.width / 4 - 2
                    anchorPoint.y: flagCheckerImage.height - 5
                    opacity: 0.0
                }

                MouseArea {
                    anchors.fill: parent
                    acceptedButtons: Qt.LeftButton | Qt.RightButton
                    onClicked: {
                         if(mouse.button & Qt.RightButton) {
                             marker_coords = map.toCoordinate(Qt.point(mouse.x,mouse.y))

                             markerIcon.opacity = 1.0
                             markerIcon.coordinate = map.toCoordinate(Qt.point(mouse.x,mouse.y))

                             markerTextLabel.text = "%1, %2".arg(marker_coords.latitude).arg(marker_coords.longitude)
                             markerTextLabel.textColor = 'darkslategray'
                         }
                    }
                }
            }
        }
    }
}
