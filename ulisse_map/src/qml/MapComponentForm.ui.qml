import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtLocation 5.6
import QtPositioning 5.6
import Qt.labs.settings 1.0
import QtQuick.Controls.Material 2.1
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.2

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
    property alias waypointPath: waypointPath
    property alias greenFlag: greenFlag

    ColorOverlay {
        id: marker
        anchors.fill: map
        source: map
        //Qt.rgba(1.0, 0.2, 0, 0.1)
        color: (settings.theme === "Light") ? "transparent" : "#FF33001A"
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

    Text {
        id: leftClick
        anchors.leftMargin: 10
        anchors.bottomMargin: my_height + 10
        anchors.left: parent.left
        anchors.bottom: parent.bottom
        color: "steelblue"
        font.weight: Font.DemiBold
        font.pointSize: 11
        textFormat: Text.StyledText
        text: 'LEFT Click: <font color="#008000">Add Waypoint</font><br>RIGHT Click: <font color="#C00000">Remove Waypoint</font>'
        opacity: mapView.pathCurrentState === pathState.creating ? 1.0 : 0.0
        z: goalAcceptRadius.z + 2
    }

    MapQuickItem {
        id: overlayText
        sourceItem: Text {
            color: 'darkslategray'
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
        border.color: 'gray'
        opacity: goalFlag.opacity == 1.0 ? goalFlag.opacity : 0.0
        z: map.z + 2
    }

    MapPolyline {
        id: ulissePath
        line.width: 1
        line.color: "#ffb300" //Material.color(Material.Amber, Material.Shade600)
        property bool firstRun: true
        property real traceSize: 1000
        z: map.z + 2
    }

    MapPolyline {
        id: waypointPath
        objectName: "waypointPath"
        line.width: 2
        line.color: (pathCurrentState === pathState.creating)
                    || (pathCurrentState === pathState.empty) ? "#ff8a65" : "#81c784"
        opacity: 0.0
        z: map.z + 1
    }

    MouseArea {
        id: mapMouseArea
        objectName: "mapMouseArea"
        anchors.fill: parent
        acceptedButtons: Qt.LeftButton | Qt.RightButton
    }
}
