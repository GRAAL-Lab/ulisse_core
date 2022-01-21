import QtQuick 2.0
import QtLocation 5.9
import QtPositioning 5.9
import QtQuick.Controls.Material 2.1
import "."

Item {
    objectName: "obstacleManager"
    //id: mapObstacleManager

    property Component obstacleComponent
    //property Component avoidancePath

    property var obstacleList: []

    Component.onCompleted: {
        obstacleComponent = Qt.createComponent("MapObstacle.qml")
        //avoidancePath = Qt.createComponent("")
        //visualizeObstacle("QML_Obstacle_1", QtPositioning.coordinate(44.0957, 9.8632), 0, 20, 10)
    }

    function visualizeObstacle(obsID, obsCoords, obsHeading, obsBBoxX, obsBBoxY) {

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


    function visualizeAvoidancePath(){

    }

}
