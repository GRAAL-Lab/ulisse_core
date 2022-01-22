import QtQuick 2.0
import QtLocation 5.9
import QtPositioning 5.9
import QtQuick.Controls.Material 2.1
import "."

MapPolygon {
    id: obstacle
    opacity: objectOpacity
    z: map.z + 4

    property real markerRadius: 1
    property color objectColor: 'red'
    property real objectOpacity: 1.0
    property var obstacleMarker
    property var objectTextOverlay
    property real lineWidth: 2
    property var coordinate: QtPositioning.coordinate(44.0956, 9.8631)
    property int timeoutSeconds: settings.visualizerTimeout
    property int countDownTimer: timeoutSeconds

    color: 'transparent'
    border.width: lineWidth
    border.color: objectColor

    property string id: "obstacleID"
    property alias coords: obstacle.coordinate
    property double heading: 0
    property double bBoxX: 0
    property double bBoxY: 0


    Component {
        id: obstacleMarkerComponent
        MapCircle {
            color: 'transparent'
            border.width: lineWidth
            border.color: objectColor
            center: coords;
            radius: markerRadius;
            opacity: objectOpacity
        }
    }

    Component {
        id: objectTextOverlayComponent
        MapQuickItem {
            z: map.z + 5
            coordinate: coords
            visible: settings.showObstacleID
            sourceItem: Item {
                Text {
                    id: overlayText
                    text: id
                    font.family: "Courier New"
                    font.pointSize: 10
                    color: objectColor
                    opacity: Math.min(0.6, objectOpacity)
                }
            }
            anchorPoint.x: overlayText.width / 2
            anchorPoint.y: overlayText.height / 2 - 15
        }
    }

    ////////////////////////////////////
    // Initializazion / deinitialization
    Component.onCompleted: {

        obstacleMarker = obstacleMarkerComponent.createObject(map);
        map.addMapItem(obstacleMarker);

        objectTextOverlay = objectTextOverlayComponent.createObject(map);
        map.addMapItem(objectTextOverlay);

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
        console.log("obstacleID: " + id + ", coords: (" + coords.latitude + "," + coords.longitude + "), heading: " + heading
                    + ", size: (" + bBoxX + ", " + bBoxY + ")")
    }

    function deregister_map_items() {
        map.removeMapItem(obstacleMarker)
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
                addonsBridgeVisualizer.deleteObstacle(id)
            }
        }
    }
}
