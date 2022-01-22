import QtQuick 2.0
import QtLocation 5.9
import QtPositioning 5.9
import QtQuick.Controls.Material 2.1
import "."

Item {
    objectName: "addonsBridgeVisualizer"

    property Component obstacleComponent
    property Component polylineComponent

    property var obstacleList: []
    property var polylineList: []

    Component.onCompleted: {
        obstacleComponent = Qt.createComponent("MapObstacle.qml")
        polylineComponent = Qt.createComponent("MapCustomPolyline.qml")

        drawObstacle("QML_Obstacle_1", QtPositioning.coordinate(44.0957, 9.8632), 0, 20, 10)
        //var polypath = GeoShape.PathType;
        //polypath.addCoordinate();
        //drawPolyline("QML_Poly_1", )
    }

    function drawObstacle(obsID, obsCoords, obsHeading, obsBBoxX, obsBBoxY) {

        for (var i = 0; i < obstacleList.length; i++) {
            if (obstacleList[i].id === obsID) {
                //console.log("Object alredy present, updating...")
                obstacleList[i].update(obsCoords, obsHeading, obsBBoxX, obsBBoxY);
                return;
            }
        }

        //console.log("New Obstacle, creating...")
        obstacleList.push(obstacleComponent.createObject(map_component, {
                                                             id: obsID,
                                                             coordinate: obsCoords,
                                                             heading: obsHeading,
                                                             bBoxX: obsBBoxX,
                                                             bBoxY: obsBBoxY
                                                         }))

        if (obstacleComponent.status != Component.Ready)
        {
            if (obstacleComponent.status == Component.Error)
                console.debug("Error: " + obstacleComponent.errorString());
            return;
        }
        map.addMapItem(obstacleList[obstacleList.length - 1])
    }

    function deleteObstacle(obsID) {

        for (var i = 0; i < obstacleList.length; i++) {
            if (obstacleList[i].id === obsID) {
                //console.log("Deleting obstacle...")
                map.removeMapItem(obstacleList[i]);
                obstacleList[i].destroy();
                // Removing the obstacle from the list
                obstacleList.splice(i, 1);
                return
            }
        }
    }


    function drawPolyline(polyID, polypath){
        for (var i = 0; i < polylineList.length; i++) {
            if (polylineList[i].id === polyID) {
                //console.log("Object alredy present, updating...")
                polylineList[i].update(polypath);
                return;
            }
        }

        //console.log("New Obstacle, creating...")
        polylineList.push(polylineComponent.createObject(map_component, {
                                                             id: polyID,
                                                             path: polypath
                                                         }))

        if (polylineComponent.status != Component.Ready)
        {
            if (polylineComponent.status == Component.Error)
                console.debug("Error: " + polylineComponent.errorString());
            return;
        }
        map.addMapItem(polylineList[polylineList.length - 1])
    }


    function deletePolyline(polyID) {

        for (var i = 0; i < polylineList.length; i++) {
            if (polylineList[i].id === polyID) {
                //console.log("Deleting obstacle...")
                map.removeMapItem(polylineList[i]);
                polylineList[i].destroy();
                // Removing the obstacle from the list
                polylineList.splice(i, 1);
                return
            }
        }
    }

}
