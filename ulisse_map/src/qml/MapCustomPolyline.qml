import QtQuick 2.0
import QtLocation 5.9
import QtPositioning 5.9
import QtQuick.Controls.Material 2.1
import "."

MapPolyline {
    id: polyline
    opacity: objectOpacity
    z: map.z + 4

    property string id: "polylineID"
    property var start: QtPositioning.coordinate(44.0956, 9.8631)
    property color objectColor: blue
    property real lineWidth: 2
    property int timeoutSeconds: settings.visualizerTimeout
    property int countDownTimer: timeoutSeconds
    property real objectOpacity: 1.0
    property var objectTextOverlay

    line.width: lineWidth
    line.color: objectColor

    Component {
        id: objectTextOverlayComponent
        MapQuickItem {
            z: map.z + 5
            coordinate: start
            visible: settings.showPolylineID
            sourceItem: Item {
                Text {
                    id: overlayText
                    text: id
                    font.family: "Courier New"
                    font.pointSize: 10
                    color: objectColor
                    opacity: objectOpacity
                    font.weight: Font.DemiBold
                }
            }
            anchorPoint.x: overlayText.width / 2
            anchorPoint.y: overlayText.height / 2 - 15
        }
    }

    ////////////////////////////////////
    // Initializazion / deinitialization
    Component.onCompleted: {

        objectTextOverlay = objectTextOverlayComponent.createObject(map);
        map.addMapItem(objectTextOverlay);

        _internalUpdate()
    }

    function update(polypath) {

        countDownTimer = timeoutSeconds
        path = polypath

        _internalUpdate()
    }

    function _internalUpdate(){

        start = path[0]
        // console.log("[MapObstacle] Obstacle Update (timeout: " + timeoutSeconds + " s)")
        console.log("Polyline ID: " + id)
    }

    function deregister_map_items() {
        map.removeMapItem(objectTextOverlay)
    }


    Timer {
        id: selfDestroyingTimer
        interval: 1000 // milliseconds
        running: true;
        repeat: true
        onTriggered: {
            countDownTimer = countDownTimer - 1;
            objectOpacity = countDownTimer/timeoutSeconds;
            if(countDownTimer == 0){
                //console.log("Obstacle '" + id + "' reached timeout.")
                deregister_map_items();
                addonsBridgeVisualizer.deletePolyline(id)
            }
        }
    }
}
