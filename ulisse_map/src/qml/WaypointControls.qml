import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import QtQuick.Controls.Styles 1.4
import QtQuick.Dialogs 1.2

RowLayout {

    property alias wpRadius: waypointRadius.text
    property alias wpButtonText: waypointsButton.text
    property alias wpButtonHighlighted: waypointsButton.highlighted
    property bool loopPath: false

    function createPath() {
        waypointsButton.text = "Send Path"
        waypointsButton.highlighted = true;
        mapView.pathCurrentState = pathState.creating;
    }

    function startPath() {
        waypointsButton.text = "Pause Path"
        waypointsButton.Material.accent = mainAccentColor;
        greenFlag.coordinate = waypointPath.path[waypointPath.pathLength() - 1];
        mapView.pathCurrentState = pathState.active;
        markerIcon.opacity = 0.4;
        cmdWrapper.startPath();
    }

    function stopPath() {
        waypointsButton.text = "Resume Path"
        mapView.pathCurrentState = pathState.stopped;
        cmdWrapper.stopPath();
    }

    function resumePath() {
        waypointsButton.text = "Pause Path"
        waypointsButton.Material.accent = mainAccentColor;
        mapView.pathCurrentState = pathState.active;
        cmdWrapper.resumePath();
    }

    function interruptPath() {
        if (mapView.pathCurrentState == pathState.active){
            stopPath();
            toast.show("Path Interrupted!", 3000);
        }
    }

    Button {
        id: waypointsButton
        Material.accent: secondaryAccentColor
        text: "Create Path"
        Layout.preferredWidth: 110

        onClicked: {
            if(waypointRadius.text !== ''){
                if (mapView.pathCurrentState === pathState.empty){
                    wpCommands.createPath();
                } else if (mapView.pathCurrentState === pathState.creating) {
                    wpCommands.startPath();
                } else if (mapView.pathCurrentState === pathState.active) {
                    wpCommands.stopPath();
                } else if (mapView.pathCurrentState === pathState.stopped) {
                    wpCommands.resumePath();
                }
            } else {
                acceptRadDialog.open();
            }
        }
    }

    Button {
        id: wpRestartButton
        Material.accent: secondaryAccentColor
        Layout.preferredWidth: 28
        enabled: (mapView.pathCurrentState === pathState.active) || (mapView.pathCurrentState === pathState.stopped)  ? true : false

        ToolTip.text: qsTr("Restart path from beginning")
        ToolTip.delay: 500
        ToolTip.timeout: 5000
        ToolTip.visible: hovered

        Image {
            id: restartIco
            anchors.fill: parent
            source: wpRestartButton.enabled ? "qrc:/images/restart-256.png" : "qrc:/images/restart-256_grey.png"
            mipmap: true
            fillMode: Image.PreserveAspectFit
            horizontalAlignment: Image.AlignHCenter
            verticalAlignment: Image.AlignVCenter
        }

        onClicked: {
            cmdWrapper.startPath();
        }
    }

    Button {
        id: wpDeleteButton
        Material.accent: Material.Red
        highlighted: true
        Layout.preferredWidth: 28
        text: "X"
        font.weight: Font.Bold
        font.pointSize: 12
        enabled: mapView.pathCurrentState === pathState.empty ? false : true

        ToolTip.text: qsTr("Delete the current path and stop")
        ToolTip.delay: 500
        ToolTip.timeout: 5000
        ToolTip.visible: hovered

        onClicked: {
            while (waypointPath.pathLength() > 0){
                map.removeMapItem(mapCircles[waypointPath.pathLength() - 1]);
                mapCircles[waypointPath.pathLength() - 1].destroy();
                waypointPath.removeCoordinate(waypointPath.pathLength() - 1);
            }

            console.log(("Destroyed Path: pathLength = %1").arg(waypointPath.pathLength()))
            waypointsButton.text = "Create Path";
            waypointsButton.highlighted = false;
            waypointsButton.Material.accent = secondaryAccentColor;
            waypointPath.opacity = 0.0;
            mapView.pathCurrentState = pathState.empty;

            cmdWrapper.cancelPath();
        }
    }

    Rectangle {
        id: waypointsSpacer
        width: buttonsColumn.width - waypointsButton.width - waypointRadius.width - wpDeleteButton.width - wpRestartButton.width - 10
        height: parent.height
        anchors.left: wpDeleteButton.right
        color: 'transparent'
    }


    TextField {
        id: waypointRadius
        objectName: "waypointRadius"
        Layout.preferredWidth: 45
        Layout.minimumWidth: 45
        Layout.maximumWidth: 45
        font.pointSize: 10
        placeholderText: "Radius"
        selectByMouse: true
        text: "5.0"
        enabled: mapView.pathCurrentState === pathState.active ? false : true

        ToolTip.text: qsTr("Acceptance Radius (meters)")
        ToolTip.delay: 500
        ToolTip.timeout: 5000
        ToolTip.visible: hovered

        anchors.left: waypointsSpacer.right
        validator: DoubleValidator {
            bottom: 0.0;
            top: 50.0;
            decimals: 1;
            notation: DoubleValidator.StandardNotation
        }
    }


}
