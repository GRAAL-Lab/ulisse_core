import QtQuick 2.0
import QtLocation 5.9
import QtPositioning 5.9
import QtQuick.Controls.Material 2.1
import "."

MapPolyline {
    id: obstacle
    opacity: objectOpacity
    z: map.z + 4

    property real markerRadius: 1
    property real headingLineLength: markerRadius * 3  // Length of the heading indicator
    property color objectColor: "red"
    property real objectOpacity: 1.0
    property real labelOpacity: 1.0
    property var obstacleMarker
    property var objectTextOverlay
    property var headingLine   // New: heading line object
    property real lineWidth: 2
    property var coordinate: QtPositioning.coordinate(44.0956, 9.8631)
    property int timeoutSeconds: settings.visualizerTimeout
    property int countDownTimer: timeoutSeconds

    //color: 'transparent'
    line.width: lineWidth
    line.color: objectColor

    property string id: "obstacleID"
    property alias coords: obstacle.coordinate
    property double heading: 0
    property double bBoxXBow: 0
    property double bBoxXStern: 0
    property double bBoxYStarboard: 0
    property double bBoxYPort: 0

    Component {
        id: obstacleMarkerComponent
        MapCustomCircle {
            //    color: 'transparent'
            line.width: lineWidth
            line.color: objectColor
            center: coords;
            radius: markerRadius;
            opacity: objectOpacity * labelOpacity
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
                    opacity: objectOpacity * labelOpacity
                    font.weight: Font.DemiBold
                }
            }
            anchorPoint.x: overlayText.width / 2
            anchorPoint.y: overlayText.height / 2 - 15
        }
    }

    // New Component for the heading line indicator
    Component {
        id: headingLineComponent
        MapPolyline {
            line.width: lineWidth
            line.color: objectColor
            opacity: objectOpacity * labelOpacity
            z: map.z + 4
            // Draw a short line from the circle center toward the heading direction.
            // The line starts at 'coords' and ends at a point computed using the heading.
            path: [
                coords,
                coords.atDistanceAndAzimuth(headingLineLength, heading)
            ]
        }
    }

    ////////////////////////////////////
    // Initialization / deinitialization
    Component.onCompleted: {
        obstacleMarker = obstacleMarkerComponent.createObject(map);
        map.addMapItem(obstacleMarker);

        objectTextOverlay = objectTextOverlayComponent.createObject(map);
        map.addMapItem(objectTextOverlay);

        // Create and add the heading line indicator.
        headingLine = headingLineComponent.createObject(map);
        map.addMapItem(headingLine);

        _internalUpdate()
    }

    function update(obsCoords, obsHeading, obsBBoxXBow, obsBBoxXStern, obsBBoxYStarboard, obsBBoxYPort, showID, color) {
        coords = obsCoords
        heading = obsHeading

        bBoxXBow = obsBBoxXBow
        bBoxXStern = obsBBoxXStern
        bBoxYStarboard = obsBBoxYStarboard
        bBoxYPort = obsBBoxYPort

        objectColor = color

        if (showID === false){
            labelOpacity = 0;
        }

        countDownTimer = timeoutSeconds
        _internalUpdate()
    }

    function _internalUpdate(){
        var obsCorners = []

        // NED coordinates
        var _top = coords.atDistanceAndAzimuth(bBoxXBow, heading)
        var _bottom = coords.atDistanceAndAzimuth(bBoxXStern, heading + 180)
        obsCorners.push(_top.atDistanceAndAzimuth(bBoxYPort, heading + 270))    // _topLeft
        obsCorners.push(_top.atDistanceAndAzimuth(bBoxYStarboard, heading + 90))   // _topRight
        obsCorners.push(_bottom.atDistanceAndAzimuth(bBoxYStarboard, heading + 90))// _bottomRight
        obsCorners.push(_bottom.atDistanceAndAzimuth(bBoxYPort, heading + 270))     // _bottomLeft
        obsCorners.push(_top.atDistanceAndAzimuth(bBoxYPort, heading + 270))        // closing point

        obstacle.path = []
        for (var i = 0; i < obsCorners.length; i++){
            obstacle.addCoordinate(obsCorners[i])
        }

        // Update the heading line's path to reflect new coordinates or heading changes.
        headingLine.path = [
            coords,
            coords.atDistanceAndAzimuth(headingLineLength, heading)
        ]
    }

    function deregister_map_items() {
        map.removeMapItem(obstacleMarker)
        map.removeMapItem(objectTextOverlay)
        map.removeMapItem(headingLine)
    }

    Timer {
        id: selfDestroyingTimer
        interval: 1000 // milliseconds
        running: true;
        repeat: true
        onTriggered: {
            countDownTimer = countDownTimer - 1;
            objectOpacity = countDownTimer / timeoutSeconds;
            if(countDownTimer == 0){
                deregister_map_items();
                addonsBridgeVisualizer.deleteObstacle(id)
            }
        }
    }
}

