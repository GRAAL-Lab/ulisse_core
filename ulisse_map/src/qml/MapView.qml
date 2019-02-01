import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtLocation 5.6
import QtPositioning 5.6
import Qt.labs.settings 1.0
import QtQuick.Controls.Material 2.1
import QtQuick.Controls.Universal 2.1
import QtQuick.Controls.Styles 1.4
import "."

RowLayout {
    spacing: 0
    property var marker_coords: QtPositioning.coordinate(44.4, 8.94)
    property bool ulisse_state_changed: false
    property real myElevation: 6
    property real panesMargin: 14

    Material.background: 'white'

    Plugin {
        id: mapPlugin
        name: "osm"
        PluginParameter {
            name: "osm.mapping.offline.directory"
            value: "/home/graal/.cache/QtLocation/5.8/tiles/osm/"
        }
    }

    ModalPopup {
        id: acceptRadDialog
        dialogTitle: "Insert an acceptance radius"

    }

    ModalPopup {
        id: speedHeadingDialog
        dialogTitle: "Insert both speed (m/s) and heading (deg)"
    }


    Rectangle {
        id: leftbarrect
        //Layout.fillWidth: true
        Layout.fillHeight: true
        Layout.minimumHeight: 150
        Layout.preferredWidth: 236
        Layout.maximumWidth: 235
        Layout.topMargin: 5
        color: 'white'

        ColumnLayout {
            id: leftbarlayout
            anchors.fill: parent
            spacing: 6
            Layout.leftMargin: 15

            Pane {
                id: statusdatarect
                Layout.alignment: Qt.AlignCenter
                Layout.preferredWidth: parent.width - panesMargin
                Material.elevation: myElevation  * 0

                ColumnLayout {
                    id: statusdatalayout
                    width: parent.width
                    Layout.preferredHeight: ulisseStateLabel.height + ulissePosLabel.height
                    spacing: 0

                    Label {
                        Layout.alignment: Qt.AlignHCenter
                        font.pointSize: 11
                        font.weight: Font.DemiBold
                        color: 'gray'
                        text: "Status"
                    }

                    LabelledText {
                        id: ulisseStateLabel
                        labelColor: Material.color(mainColor, Material.Shade700)
                        label: "Ulisse State"
                        textColor: 'lightgray'
                        text: "%1".arg(fbkUpdater.vehicle_state)
                        textBoldness: Font.DemiBold
                        onTextChanged: {
                            if(ulisse_state_changed){
                                ulisseStateLabel.textColor = 'darkslategray';
                            }
                            ulisse_state_changed = true;
                        }
                    }

                    LabelledText {
                        id: ulissePosLabel
                        labelColor: Material.color(mainColor, Material.Shade700)
                        label: "Ulisse Coordinates"
                        textColor: 'darkslategray'
                        text: "%1, %2".arg(fbkUpdater.ulisse_pos.latitude).arg(fbkUpdater.ulisse_pos.longitude)
                    }

                }
            }

            Pane {
                id: goaldatarect
                Layout.alignment: Qt.AlignCenter
                Layout.preferredWidth: parent.width - panesMargin
                Material.elevation: myElevation  * 0

                ColumnLayout {
                    id: goaldatalayout
                    width: parent.width
                    Layout.preferredHeight: goalTextLabel.height
                    spacing: 0

                    Label {
                        Layout.alignment: Qt.AlignHCenter
                        font.pointSize: 11
                        font.weight: Font.DemiBold
                        color: 'gray'
                        text: "Goal"
                    }

                    LabelledText {
                        id: goalTextLabel
                        labelColor: Material.color(mainColor, Material.Shade700)
                        label: "Goal Coordinates"
                        textColor: 'darkslategray'
                        text: "%1, %2".arg(fbkUpdater.goal_pos.latitude).arg(fbkUpdater.goal_pos.longitude)
                    }

                    LabelledText {
                        id: goalDistLabel
                        labelColor: Material.color(mainColor, Material.Shade700)
                        label: "Distance to Target"
                        textColor: 'darkslategray'
                        text: "%1 (m)".arg(fbkUpdater.goal_distance)

                    }
                }
            }

            Pane {
                id: infodatarect
                Layout.alignment: Qt.AlignCenter
                Layout.preferredWidth: parent.width - panesMargin
                //Layout.preferredHeight: infodatalayout.height
                Material.elevation: myElevation * 0

                ColumnLayout {
                    id: infodatalayout
                    width: parent.width
                    Layout.preferredHeight: markerTextLabel.height + goalTextLabel.height

                    LabelledText {
                        id: markerTextLabel
                        labelColor: Material.color(Material.Red, Material.Shade800)
                        label: "Marker Coordinates"
                        textColor: 'lightgray'
                        text: "Right click on map"
                    }
                }
            }

            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.minimumHeight: 14
                color: 'transparent'
                Text {
                    //anchors.centerIn: parent
                    width: parent.width
                    font.pointSize: 8
                    color: 'darkslategray'
                    text: "(Right click to set marker)"
                    horizontalAlignment: Text.AlignHCenter
                }
            }


            CommandPane {
                id: commandRect
                Layout.alignment: Qt.AlignCenter
                Layout.preferredWidth: parent.width - panesMargin
                Layout.bottomMargin: 10
                Material.elevation: myElevation
                //Material.background: Material.color(Material.BlueGrey, Material.Shade50)
            }


        }
    }

    Column {
        Layout.fillWidth: true
        Layout.fillHeight: true
        Layout.minimumWidth: 300
        Layout.preferredWidth: 500
        Layout.preferredHeight: 500

        Map {


            id: map
            width: parent.width
            height: parent.height - bottomToolbar.height
            plugin: mapPlugin
            center: QtPositioning.coordinate(44.393, 8.945) // Genoa

            zoomLevel: 14//(maximumZoomLevel - minimumZoomLevel)/2

            MapRuler {
                id: ruler
                anchors.fill: parent
            }

            MapSliders {
                id: sliders
                z: map.z + 3
                mapSource: map
                edge: Qt.RightEdge
            }

            onCenterChanged:{
                ruler.rulerTimer.restart()
                /*if (map.followme)
                    if (map.center !== positionSource.position.coordinate) map.followme = false*/
            }

            onZoomLevelChanged:{
                ruler.rulerTimer.restart()
                //if (map.followme) map.center = positionSource.position.coordinate
            }

            onWidthChanged:{
                ruler.rulerTimer.restart()
            }

            onHeightChanged:{
                ruler.rulerTimer.restart()
            }

            MapQuickItem {
                id:markerIcon
                sourceItem: Image{
                    id: markerImage
                    width: 32; height: 32
                    source: 'qrc:/images/map-marker-64.png'

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
                    source: 'qrc:/images/catamaran_icon_64_sat.png'
                    transform: Rotation { origin.x: ulisseImage.width / 2 ; origin.y: ulisseImage.height / 2; angle: fbkUpdater.ulisse_yaw_deg }
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
                    width: 72; height: 72
                    source: 'qrc:/images/flag_checker.png'
                }
                coordinate: QtPositioning.coordinate(fbkUpdater.goal_pos.latitude, fbkUpdater.goal_pos.longitude)
                anchorPoint.x: flagCheckerImage.width / 2
                anchorPoint.y: flagCheckerImage.height / 2
                opacity: 0.0
            }


            MapPolyline {
                id: ulissePath
                line.width: 1
                line.color: Material.color(Material.Amber, Material.Shade300)
                property bool firstRun: true

                Timer {
                    interval: 500; running: true; repeat: true

                    onTriggered: {
                        if (ulissePath.firstRun) {
                            ulissePath.addCoordinate(fbkUpdater.ulisse_pos)
                            ulissePath.firstRun = false;
                        }

                        var lastCoord = ulissePath.coordinateAt(ulissePath.pathLength() - 1);
                        var distToNext = lastCoord.distanceTo(fbkUpdater.ulisse_pos);
                        //toast.show("Distance %1".arg(distToNext));
                        if (distToNext > 0.5){
                            ulissePath.addCoordinate(fbkUpdater.ulisse_pos)
                            if(ulissePath.pathLength() > 500) {
                                ulissePath.removeCoordinate(0);
                            }
                        }
                    }
                }
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

        Rectangle {

            id: bottomToolbar
            width: parent.width
            height: clearPathButton.height;
            color: 'gainsboro'

            Button {
                id:clearPathButton
                anchors.right: parent.right
                anchors.bottom: parent.bottom
                anchors.rightMargin: 5
                text: "Clear path"
                highlighted: true
                Material.accent: Material.Cyan

                onClicked: {
                    ulissePath.path = [];
                    ulissePath.firstRun = true;
                }
            }
        }
    }
}
