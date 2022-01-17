import QtQuick 2.0
import QtLocation 5.9
import QtPositioning 5.9
import QtQuick.Controls.Material 2.1
import "."

MapPolygon {
    id: obstacle
    opacity: 1.0
    z: map.z + 4

    property real markerRadius: 1
    property color obstacleColor: 'red'
    property real lineWidth: 2
    property var coordinate: QtPositioning.coordinate(44.0956, 9.8631)

    color: 'transparent'
    border.width: lineWidth
    border.color: obstacleColor

    property string id: "obstacleID"
    property alias coords: obstacle.coordinate
    property double heading: 0
    property double bBoxX: 0
    property double bBoxY: 0

    property int objectTimeout: 30

    Component {
        id: obstacleMarkerComponent
        MapCircle {
            color: 'transparent'
            border.width: lineWidth
            border.color: obstacleColor
            center: coords;
            radius: markerRadius;
        }
    }

    Component {
        id: obstacleTextOverlayComponent
        MapQuickItem {
            opacity: 0
            z: map.z + 5
            sourceItem: Item {
                Text {
                    id: osbtacleText
                    text: id
                    font.family: "Helvetica"
                    font.pointSize: 12
                    color: obstacleColor
                }

                transform: [
                    Rotation {
                        origin.x: osbtacleText.width / 2
                        origin.y: osbtacleText.height / 2
                        angle: map.bearing
                    }
                ]
            }
            anchorPoint.x: 5 //map_marker_letter.sourceItem.width / 2
            anchorPoint.y: 5 //map_marker_letter.sourceItem.height / 2

        }
    }

    ////////////////////////////////////
    // Initializazion / deinitialization
    Component.onCompleted: {

        console.log("[MapObstacle] Adding Obstacle")
        console.log("ID: " + id + ", coords: (" + coords.latitude + "," + coords.longitude + "), heading: " + heading
                    + ", size: (" + bBoxX + ", " + bBoxY + ")")

        var obstacleMarker = obstacleMarkerComponent.createObject(map);
        map.addMapItem(obstacleMarker);

        var obstacleTextOverlay = obstacleTextOverlayComponent.createObject(map);
        map.addMapItem(obstacleTextOverlay);

        var obsCorners = [];
        var _top = coords.atDistanceAndAzimuth(bBoxX/2.0, heading)
        //var _right = coords.atDistanceAndAzimuth(bBoxY/2.0, heading + 90)
        var _bottom = coords.atDistanceAndAzimuth(bBoxX/2.0, heading + 180)
        //var _left = coords.atDistanceAndAzimuth(bBoxY/2.0, heading + 270)

        obsCorners.push(_top.atDistanceAndAzimuth(bBoxY/2.0, heading + 270))    // _topLeft
        obsCorners.push(_top.atDistanceAndAzimuth(bBoxY/2.0, heading + 90))     // _topRight
        obsCorners.push(_bottom.atDistanceAndAzimuth(bBoxY/2.0, heading + 90))  // _bottomRight
        obsCorners.push(_bottom.atDistanceAndAzimuth(bBoxY/2.0, heading + 270)) // _bottomLeft

        for (var i = 0; i < obsCorners.length; i++){
            obstacle.addCoordinate(obsCorners[i])
        }

        //obstacle.addCoordinate(QtPositioning.coordinate(44.0957, 9.8630));
        //obstacle.addCoordinate(QtPositioning.coordinate(44.0957, 9.8630));
        //obstacle.addCoordinate(QtPositioning.coordinate(44.0955, 9.8630));
        //obstacle.addCoordinate(QtPositioning.coordinate(44.0955, 9.8632));

    }

    function add_to_map() {
        /*id = obsID;
        heading = obsHeading;*/
        //map.addMapItem(obstacleMarker);
    }

    function deregister_map_items() {
        //map.removeMapItem(_canvas)
    }

}
