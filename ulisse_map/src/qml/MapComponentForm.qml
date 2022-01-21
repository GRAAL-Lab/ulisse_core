import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtLocation 5.6
import QtPositioning 5.6
import Qt.labs.settings 1.0
import QtQuick.Controls.Material 2.1
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.2
import "."

Map {
    id: map_component
    property real markerIconOpacity: markerIcon.opacity
    property alias ulisseOverlayTextOpacity: ulisseOverlayText.opacity
    property alias markerIcon: markerIcon
    property alias mapMouseArea: mapMouseArea
    property alias ulissePath: ulissePath
    property alias compass: compass
    property alias currentArrow: currentArrow
    property alias currentLabel: currentLabel
    property alias ulisseIcon: ulisseIcon
    property alias ulisseGPSIcon: ulisseGPSIcon
    property alias goalFlag: goalFlag
    property alias ruler: ruler
    property alias greenFlag: greenFlag
    property alias editCircle: editCircle
    property alias sliderz: sliders.z
    property alias mapTextOverlay: mapTextOverlay

    Text {
        id: mapTextOverlay

        font.pointSize: 12
        //style: Text.Outline
        //styleColor: "white"
        color: grey
        horizontalAlignment: Text.AlignHCenter
        width: parent.width
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 110
        text: "Overlay Text"
        font.weight: Font.DemiBold
        z: map.z + 4
        visible: false
    }

    MapRuler {
        id: ruler
        anchors.left: map.left
        anchors.top: map.top
        anchors.topMargin: 10
        anchors.leftMargin: 20
        width: 60

        layer.enabled: true
        layer.effect: Glow {
            radius: 4
            samples: 10
            color: lightergrey

        }
    }

    MapSliders {
        id: sliders
        z: map.z + 5
        mapSource: map
        edge: Qt.RightEdge
    }

    Image {
        id: compass
        source: 'qrc:/images/compass_icon.svg'
        width: 42
        height: 42
        mipmap: true
        z: map.z + 2

        anchors.right: sliders.left
        anchors.top: parent.top
        anchors.rightMargin: 15
        anchors.topMargin: 20
    }

    Item{
        id: itemcurrent
        anchors.right: compass.left
        anchors.top: parent.top
        anchors.rightMargin: 15
        anchors.topMargin: 20
        width: 48
        height: 42
        z: map.z + 5

        Image {
            id: currentArrow
            mipmap: true
            antialiasing: true
            source: 'qrc:/images/windarrow.png'

            layer.enabled: true
            layer.effect: Glow {
                radius: 4
                samples: 10
                color: lightergrey

            }
        }

        Label {
            id: currentLabel
            color: "#004EAE"
            width: parent.width
            horizontalAlignment: Text.AlignHCenter
            anchors.left: currentArrow.left
            anchors.top: currentArrow.bottom


            //style: Text.Outline
            //styleColor: "white"

            layer.enabled: true
            layer.effect: Glow {
                radius: 4
                samples: 10
                color: lightergrey

            }
        }
    }

    MapQuickItem {
        id: markerIcon
        sourceItem: Image {
            id: markerImage
            width: 32
            height: 32
            source: 'qrc:/images/map-marker-64.png'
        }
        //coordinate: map.center
        z: map.z + 2
        anchorPoint.x: markerImage.width / 2
        anchorPoint.y: markerImage.height
        opacity: 1.0
    }

    MapQuickItem {
        id: centroidIcon
        sourceItem: Text {
            id: centroidText
            color: "white"
            text: "+ CENTROID"
            font.family: "Courier New"
            layer.enabled: true

        }

        coordinate: fbkUpdater.centroid
        z: map.z + 1
        /*anchorPoint.x: centroidText.width / 2
        anchorPoint.y: centroidText.height*/
        opacity: 0.4
        visible: settings.showCentroid
    }

    MapQuickItem {
        id: ulisseIcon
        sourceItem: Image {
            id: ulisseImage
            width: 38
            height: 38
            source: 'qrc:/images/catamaran_icon_64_sat.png'
        }
        coordinate: fbkUpdater.ulisse_pos
        anchorPoint.x: ulisseImage.width / 2
        anchorPoint.y: ulisseImage.height / 2
        z: map.z + 2
    }

    MapQuickItem {
        id: ulisseGPSIcon
        sourceItem: Image {
            id: ulisseGPSImage
            width: 38
            height: 38
            opacity: 0.5
            visible: gpsIconCBox.checked
            source: 'qrc:/images/catamaran_icon_64_sat.png'
        }
        coordinate: fbkUpdater.gps_pos
        anchorPoint.x: ulisseImage.width / 2
        anchorPoint.y: ulisseImage.height / 2
        z: map.z + 2
    }

    MapQuickItem {
        id: goalFlag
        objectName: "goalFlag"
        sourceItem: Image {
            id: flagCheckerImage
            width: 72
            height: 72
            source: 'qrc:/images/flag_checker.png'
        }
        coordinate: fbkUpdater.goal_pos
        anchorPoint.x: flagCheckerImage.width / 2
        anchorPoint.y: flagCheckerImage.height / 2
        z: goalAcceptRadius.z + 2
        opacity: 0.0
    }

    MapQuickItem {
        id: greenFlag
        sourceItem: Image {
            id: greenFlagImage
            width: 72
            height: 72
            source: 'qrc:/images/flag_green.png'
        }

        anchorPoint.x: greenFlagImage.width / 2
        anchorPoint.y: greenFlagImage.height / 2
        z: goalAcceptRadius.z + 1
        opacity: ((mapView.pathCurrentState === pathState.active)
                  || (mapView.pathCurrentState === pathState.stopped) ? 1.0 : 0.0)
    }

    MapQuickItem {
        id: ulisseOverlayText
        sourceItem: Text {
            color: 'darkslategrey'
            text: "Surge: " + fbkUpdater.ulisse_surge + " m/s\nHeading: "
                  + fbkUpdater.ulisse_rpy_deg.z.toFixed(2) + "°"
            //style: Text.Outline
            //styleColor: "white"
            //font.weight: Font.DemiBold

            layer.enabled: true
            layer.effect: Glow {
                radius: 4
                samples: 10
                color: lightergrey

            }
        }
        coordinate: fbkUpdater.ulisse_pos
        anchorPoint.x: -ulisseImage.width / 2
        anchorPoint.y: -ulisseImage.height / 2
        z: map.z + 3
        visible: settings.showStatusOverlay
    }

    MapCircle {
        id: goalAcceptRadius
        center: goalFlag.coordinate
        radius: fbkUpdater.accept_radius
        color: 'transparent'
        border.width: 1
        border.color: grey
        opacity: goalFlag.opacity == 1.0 ? goalFlag.opacity : 0.0
        z: map.z + 2
    }

    MapCircle {
        id: editCircle
        center: goalFlag.coordinate
        radius: 5
        color: red
        border.width: 1
        border.color: grey
        opacity: 0
        z: map.z + 2
    }

    MapPolyline {
        id: ulissePath
        line.width: 1
        line.color: orange
        property bool firstRun: true
        property real traceSize: 1000
        z: map.z + 2
    }

    MouseArea {
        id: mapMouseArea
        objectName: "mapMouseArea"
        anchors.fill: parent
        acceptedButtons: Qt.LeftButton | Qt.RightButton
    }
}
