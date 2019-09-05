import QtQuick 2.11
import QtQuick.Controls 2.4
import QtQuick.Controls.Material 2.4
import QtQuick.Controls.Universal 2.4
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.11
import QtQuick.Window 2.4
import QtLocation 5.11
import QtPositioning 5.11
import Qt.labs.settings 1.0
import QtGraphicalEffects 1.0
MapQuickItem {

    property alias source: img.source
    id: root
    opacity: 0
    z: map.z + 2
    sourceItem: Item {
        width: 40
        height: 40
        Image {
            id: img
            width: 40
            height: 40
        }
    }
    anchorPoint.x: 20
    anchorPoint.y: 20
}
