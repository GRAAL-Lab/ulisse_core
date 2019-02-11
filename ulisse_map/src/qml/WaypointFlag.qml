import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtLocation 5.6
import QtPositioning 5.6

MapQuickItem {
    //id: goalFlag
    //objectName: "goalFlag"
    sourceItem: Image{
        id: flagGreenImage
        width: 72; height: 72
        source: 'qrc:/images/flag_green.png'
    }
    //coordinate: QtPositioning.coordinate(fbkUpdater.goal_pos.latitude, fbkUpdater.goal_pos.longitude)
    anchorPoint.x: flagGreenImage.width / 2
    anchorPoint.y: flagGreenImage.height / 2
    z: map.z + 1

}
