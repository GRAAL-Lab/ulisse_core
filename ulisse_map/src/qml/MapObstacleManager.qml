import QtQuick 2.0
import QtLocation 5.9
import QtPositioning 5.9
import QtQuick.Controls.Material 2.1
import "."

Item {
    id: mapObstacleManagerItem

    property Component obstacleComponent

    property var obstacleList: []

    Component.onCompleted: {
        obstacleComponent = Qt.createComponent("MapObstacle.qml")

        visualizeObstacle("QML_Obstacle_1", QtPositioning.coordinate(44.0957, 9.8632), 0, 20, 10)
        visualizeObstacle("QML_Obstacle_2", QtPositioning.coordinate(44.0955, 9.8630), 90, 10, 5)

        visualizeObstacle("QML_Obstacle_2", QtPositioning.coordinate(44.0955, 9.8630), 90, 10, 10)

        // CALL FUNCTION FROM C++ IN A TOPIC
    }

    function visualizeObstacle(obsID, obsCoords, obsHeading, obsBBoxX, obsBBoxY) {

        for (var i = 0; i < obstacleList.length; i++) {
            if(obstacleList[i].id === obsID) {
                console.log("Object alredy present, updating...")
                obstacleList[i].update(obsCoords, obsHeading, obsBBoxX, obsBBoxY);
                return;
            }
        }

        console.log("New Obstacle, creating...")

        obstacleList.push(
                    obstacleComponent.createObject(map_component, {id: obsID, coordinate: obsCoords, heading: obsHeading, bBoxX: obsBBoxX, bBoxY: obsBBoxY})
                    )

        if( obstacleComponent.status != Component.Ready )
        {
            if( obstacleComponent.status == Component.Error )
                console.debug("Error:"+ obstacleComponent.errorString() );
            return;
        }
        map.addMapItem(obstacleList[obstacleList.length - 1])
    }

    // PROVIDE OBJECT DELETION (?)

}
