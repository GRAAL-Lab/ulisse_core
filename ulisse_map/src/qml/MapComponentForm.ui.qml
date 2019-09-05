import QtQuick 2.11
import QtQuick.Controls 2.4
import QtQuick.Controls.Material 2.4
import QtQuick.Controls.Universal 2.4
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.11
import QtQuick.Window 2.4
import QtLocation 5.11
import QtPositioning 5.11
import Qt.labs.settings 1.0
import QtGraphicalEffects 1.0
import "."

Map {
    id: map_component
    property real markerIconOpacity: markerIcon.opacity
    property alias overlayTextOpacity: overlayText.opacity
    property alias markerIcon: markerIcon
    property alias mapMouseArea: mapMouseArea
    property alias ulissePath: ulissePath
    property alias marker: marker
    property alias compass: compass
    property alias ulisseIcon: ulisseIcon
    property alias goalFlag: goalFlag
    property alias ruler: ruler
    property alias greenFlag: greenFlag
    property alias editCircle: editCircle
    property alias sliderz: sliders.z

    ColorOverlay {
        id: marker
        anchors.fill: map
        source: map
        //Qt.rgba(1.0, 0.2, 0, 0.1)
        color: (settings.theme === "Light") ? "transparent" : cyan
    }

    MapRuler {
        id: ruler
        anchors.right: compass.left
        anchors.top: parent.top
        anchors.topMargin: 10
        anchors.rightMargin: 20
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
        anchorPoint.y: markerImage.height / 2
        opacity: 0.0
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
        id: overlayText
        sourceItem: Text {
            color: 'darkslategrey'
            text: "Surge: " + fbkUpdater.ulisse_surge + " m/s\nHeading: "
                  + fbkUpdater.ulisse_yaw_deg + "°"
        }
        coordinate: fbkUpdater.ulisse_pos
        anchorPoint.x: -ulisseImage.width / 2
        anchorPoint.y: -ulisseImage.height / 2
        z: map.z + 3
        opacity: 0.0
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

/*##^##
Designer {
    D{i:0;autoSize:true;height:480;width:640}
}
##^##*/
