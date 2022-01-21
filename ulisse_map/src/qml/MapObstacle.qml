import QtQuick 2.0
import QtLocation 5.9
import QtPositioning 5.9
import QtQuick.Controls.Material 2.1
import "."

MapPolygon {
    id: obstacle
    opacity: obstacleOpacity
    z: map.z + 4

    property real markerRadius: 1
    property color obstacleColor: 'red'
    property real lineWidth: 2
    property var coordinate: QtPositioning.coordinate(44.0956, 9.8631)
    property int timeoutSeconds: settings.obstacleTimeout
    property int countDownTimer: timeoutSeconds

    color: 'transparent'
    border.width: lineWidth
    border.color: obstacleColor

    property string id: "obstacleID"
    property alias coords: obstacle.coordinate
    property double heading: 0
    property double bBoxX: 0
    property double bBoxY: 0

    property int objectTimeout: 30
    property real obstacleOpacity: 1.0

    property var obstacleMarker
    property var obstacleTextOverlay

    Component {
        id: obstacleMarkerComponent
        MapCircle {
            color: 'transparent'
            border.width: lineWidth
            border.color: obstacleColor
            center: coords;
            radius: markerRadius;
            opacity: obstacleOpacity
        }
    }

    Component {
        id: obstacleTextOverlayComponent
        MapQuickItem {
            z: map.z + 5
            coordinate: coords
            visible: settings.showObstacleID
            sourceItem: Item {
                Text {
                    id: osbtacleText
                    text: id
                    font.family: "Courier New"
                    font.pointSize: 10
                    color: obstacleColor
                    opacity: Math.min(0.6, obstacleOpacity)
                }
            }
            anchorPoint.x: osbtacleText.width / 2
            anchorPoint.y: osbtacleText.height / 2 - 15
        }
    }

    ////////////////////////////////////
    // Initializazion / deinitialization
    Component.onCompleted: {

        obstacleMarker = obstacleMarkerComponent.createObject(map);
        map.addMapItem(obstacleMarker);

        obstacleTextOverlay = obstacleTextOverlayComponent.createObject(map);
        map.addMapItem(obstacleTextOverlay);

        _internalUpdate()

    }

    function update(obsCoords, obsHeading, obsBBoxX, obsBBoxY) {
        coords = obsCoords
        heading = obsHeading
        bBoxX = obsBBoxX
        bBoxY = obsBBoxY

        countDownTimer = timeoutSeconds
        _internalUpdate()
    }

    function _internalUpdate(){
        var obsCorners = []
        var _top = coords.atDistanceAndAzimuth(bBoxX/2.0, heading)
        //var _right = coords.atDistanceAndAzimuth(bBoxY/2.0, heading + 90)
        var _bottom = coords.atDistanceAndAzimuth(bBoxX/2.0, heading + 180)
        //var _left = coords.atDistanceAndAzimuth(bBoxY/2.0, heading + 270)

        obsCorners.push(_top.atDistanceAndAzimuth(bBoxY/2.0, heading + 270))    // _topLeft
        obsCorners.push(_top.atDistanceAndAzimuth(bBoxY/2.0, heading + 90))     // _topRight
        obsCorners.push(_bottom.atDistanceAndAzimuth(bBoxY/2.0, heading + 90))  // _bottomRight
        obsCorners.push(_bottom.atDistanceAndAzimuth(bBoxY/2.0, heading + 270)) // _bottomLeft

        obstacle.path = []
        for (var i = 0; i < obsCorners.length; i++){
            obstacle.addCoordinate(obsCorners[i])
        }

        // console.log("[MapObstacle] Obstacle Update (timeout: " + timeoutSeconds + " s)")
        console.log("ID: " + id + ", coords: (" + coords.latitude + "," + coords.longitude + "), heading: " + heading
                    + ", size: (" + bBoxX + ", " + bBoxY + ")")
    }

    function deregister_map_items() {
        map.removeMapItem(obstacleMarker)
        map.removeMapItem(obstacleTextOverlay)
    }


    Timer {
        id: selfDestroyingTimer
        interval: 1000 // milliseconds
        running: true;
        repeat: true
        onTriggered: {
            countDownTimer = countDownTimer - 1;
            obstacleOpacity = countDownTimer/timeoutSeconds;
            if(countDownTimer == 0){
                //console.log("Obstacle '" + id + "' reached timeout.")
                deregister_map_items();
                mapObstacleManager.deleteObstacle(id)
                //obstacle.visible = 0;
            }
        }
    }
}
