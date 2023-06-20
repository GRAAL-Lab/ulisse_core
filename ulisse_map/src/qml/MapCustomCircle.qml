import QtQuick 2.0
import QtLocation 5.9
import QtPositioning 5.9
import QtQuick.Controls.Material 2.1
import "."

//id: goalAcceptRadius
//center: goalFlag.coordinate
//radius: fbkUpdater.accept_radius
//color: 'transparent'
//border.width: 1
//border.color: grey
//opacity: goalFlag.opacity == 1.0 ? goalFlag.opacity : 0.0
//z: map.z + 2

MapPolyline {
    id: mapCustomCircle

    property string id: ""
    property var center: QtPositioning.coordinate()
    property real radius: 0.0

    function update() {
        //console.log("Circle Center: " + center.latitude + "," + center.longitude + "), radius: " + radius + "\n" +
        //            "visible: " + visible + ", opacity: " + opacity)

        var numSamples = 36;
        var sampledPoints = [];
        var θ_step = 360.0/numSamples;

        path = []
        for (var i = 0; i <= numSamples; i++){
            addCoordinate(center.atDistanceAndAzimuth(radius, θ_step * i))
        }
    }

    onRadiusChanged: {
        update()
    }

    onCenterChanged: {
        update();
    }

    onOpacityChanged: {
        update();
    }
}

