import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtLocation 5.6
import QtPositioning 5.6
import Qt.labs.settings 1.0
import QtQuick.Controls.Material 2.1
import QtQuick.Controls.Universal 2.1
import QtQuick.Controls.Styles 1.4
import QtGraphicalEffects 1.0
import "."

RowLayout {
    spacing: 0
    property var marker_coords: QtPositioning.coordinate(44.4, 8.94)
    property bool ulisse_state_changed: false
    property real myElevation: 6
    property real panesMargin: 14

    //Material.background: 'white'

    Plugin {
        id: mapPlugin
        name: "osm"
        /*PluginParameter {
            name: "osm.mapping.offline.directory"
            value: "/home/graal/.cache/QtLocation/5.8/tiles/osm/"
        }*/
    }

    ModalPopup {
        id: acceptRadDialog
        dialogTitle: "Insert an acceptance radius"

    }

    ModalPopup {
        id: speedHeadingDialog
        dialogTitle: "Insert both speed (m/s) and heading (deg)"
    }


    MapSidebar {
        id: mapsidebar
        //Layout.fillWidth: true
        Layout.fillHeight: true
        Layout.minimumHeight: 150
        Layout.preferredWidth: 236
        Layout.maximumWidth: 235
        Layout.topMargin: 5
        //color: 'white'

    }

    property real altezzaScrittaDemmerda: 17

    Rectangle {
        id: mapContainer
        Layout.fillWidth: true
        Layout.fillHeight: true
        Layout.minimumWidth: 300
        Layout.preferredWidth: 500
        Layout.preferredHeight: 500


        Map {
            id: map
            width: parent.width
            height: parent.height - bottomToolbar.height + altezzaScrittaDemmerda
            plugin: mapPlugin
            center: QtPositioning.coordinate(44.393, 8.945) // Genoa

            zoomLevel: 14//(maximumZoomLevel - minimumZoomLevel)/2

            ColorOverlay {
                anchors.fill: map
                source: map
                color: (settings.theme === "Light") ? "transparent" : Qt.rgba(1.0, 0.2, 0, 0.1)
            }

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
            }

            onZoomLevelChanged:{
                ruler.rulerTimer.restart()
            }

            onWidthChanged:{
                ruler.rulerTimer.restart()
            }

            onHeightChanged:{
                ruler.rulerTimer.restart()
            }

            Image {
                id: compass
                source: 'qrc:/images/compass_icon.svg'
                width: 42
                height: 42
                mipmap: true
                z: map.z + 2

                anchors.right: parent.right
                anchors.top: parent.top
                anchors.rightMargin: 15
                anchors.topMargin: 20

                transform: [ Rotation {
                        origin.x: compass.width / 2 ;
                        origin.y: compass.height / 2;
                        angle: 180.0 - map.bearing
                    },
                    Rotation {
                        origin.x: ulisseImage.width / 2 ;
                        origin.y: ulisseImage.height / 2;
                        angle: map.tilt
                        axis.x: 1
                        axis.y: 0
                        axis.z: 0
                    } ]

            }

            MapQuickItem {
                id:markerIcon
                sourceItem: Image{
                    id: markerImage
                    width: 32; height: 32
                    source: 'qrc:/images/map-marker-64.png'

                }
                coordinate: map.center
                z: ulisseIcon.z + 1
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
                    //mipmap: true
                    transform: [
                        Rotation {
                            origin.x: ulisseImage.width / 2 ;
                            origin.y: ulisseImage.height / 2;
                            angle: fbkUpdater.ulisse_yaw_deg - map.bearing
                        },
                        Rotation {
                            origin.x: ulisseImage.width / 2 ;
                            origin.y: ulisseImage.height / 2;
                            angle: map.tilt
                            axis.x: 1
                            axis.y: 0
                            axis.z: 0
                        }
                    ]
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
                z: goalAcceptRadius.z + 1
                opacity: 0.0

            }

            MapQuickItem {
                id: overlayText
                sourceItem: Text {
                    color: 'darkslategray'
                    text: "Speed: %1 m/s\nHeading: %2°".arg(fbkUpdater.ulisse_speed).arg(fbkUpdater.ulisse_yaw_deg)
                }
                coordinate: QtPositioning.coordinate(fbkUpdater.ulisse_pos.latitude, fbkUpdater.ulisse_pos.longitude)
                anchorPoint.x: - ulisseImage.width / 2
                anchorPoint.y: - ulisseImage.height / 2
                z: map.z + 3
                opacity: 0.0
            }

            MapCircle {
                id: goalAcceptRadius
                center: goalFlag.coordinate
                radius: fbkUpdater.accept_radius
                color: 'transparent'
                border.width: 1
                border.color: 'lightgray'
                opacity: goalFlag.opacity == 1.0 ? goalFlag.opacity : 0.0
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

                        mapsidebar.markerText = "%1, %2".arg(marker_coords.latitude).arg(marker_coords.longitude)
                        mapsidebar.markerTextColor = Material.foreground
                    }
                }
            }
        }

        Rectangle {

            id: bottomToolbar
            width: parent.width
            height: clearPathButton.height
            color: Material.background
            anchors.bottom: parent.bottom

            RowLayout {
                width: parent.width

                Button {
                    id:recenterButton
                    text: "Recenter"
                    highlighted: true
                    Material.accent: secondaryAccentColor
                    Layout.leftMargin: 5
                    onClicked: {
                        map.center = ulisseIcon.coordinate;
                    }
                }

                CheckBox {
                    id: followMeCheckbox
                    text: "Follow vehicle"
                    anchors.left: recenterButton.right
                    Material.accent: secondaryAccentColor
                    checked: false

                    Timer {
                        id: followMeTimer
                        interval: 250; running: false; repeat: true

                        onTriggered: {
                            map.center = ulisseIcon.coordinate;
                            //console.log("Triggered %1".arg(followMeCheckbox.checked))
                        }
                    }

                    onCheckStateChanged: {
                        if (checked === true){
                            followMeTimer.start()
                        } else {
                            followMeTimer.stop()
                        }
                    }

                }

                CheckBox {
                    id: overlayStatusCbox
                    text: "Show Overlay"
                    anchors.left: followMeCheckbox.right
                    Material.accent: secondaryAccentColor
                    checked: false

                    onCheckStateChanged: {
                        if (checked === true){
                            overlayText.opacity = 1.0;
                        } else {
                            overlayText.opacity = 0.0;
                        }
                    }

                }

                Button {
                    id:clearPathButton
                    Layout.rightMargin: 5
                    text: "Clear path"
                    highlighted: true
                    Material.accent: mainAccentColor
                    Layout.alignment: Qt.AlignRight

                    onClicked: {
                        ulissePath.path = [];
                        ulissePath.firstRun = true;
                    }
                }
            }
        }
    }
}
