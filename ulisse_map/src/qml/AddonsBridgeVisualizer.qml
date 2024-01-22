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

        //drawObstacle("QML_Obstacle_1", QtPositioning.coordinate(44.0957, 9.8632), 0, 20, 10)

        //var sampleCoordList = [
        //            QtPositioning.coordinate(44.0957, 9.8632),
        //            QtPositioning.coordinate(44.0956, 9.8633) ]
        //drawPolyline("QML_Poly_1", sampleCoordList)
    }

    function drawObstacle(obsID, obsCoords, obsHeading, obsBBoxX, obsBBoxY, obsShowID, obsColor) {
        // Checking if object already exists in the manager's list
        for (var i = 0; i < obstacleList.length; i++) {
            if (obstacleList[i].id === obsID) {
                obstacleList[i].update(obsCoords, obsHeading, obsBBoxX, obsBBoxY, obsShowID, obsColor);
                return;
            }
        }

        // Otherwise create it
        obstacleList.push(obstacleComponent.createObject(map_component, {
                                                             id: obsID,
                                                             coordinate: obsCoords,
                                                             heading: obsHeading,
                                                             bBoxX: obsBBoxX,
                                                             bBoxY: obsBBoxY,
                                                             showID: obsShowID,
                                                             objectColor: obsColor
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
                // Removing the obstacle from the manager list
                obstacleList.splice(i, 1);
                return
            }
        }
    }


    function drawPolyline(polyID, polypath){
        // Checking if object already exists in the manager's list
        for (var i = 0; i < polylineList.length; i++) {
            if (polylineList[i].id === polyID) {
                polylineList[i].update(polypath);
                return;
            }
        }

        // Otherwise create it
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
                map.removeMapItem(polylineList[i]);
                polylineList[i].destroy();
                // Removing the polyline from manager the list
                polylineList.splice(i, 1);
                return
            }
        }
    }
}
