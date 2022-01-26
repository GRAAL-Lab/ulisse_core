import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import "."

Rectangle {

    id: actionView
    property var titlesize: 15
    property var labelsize: 13
    color: Material.background

    Pane {
        anchors.fill: parent
        anchors.margins: 20

        ColumnLayout {

            Label {
                objectName: "actionLabelObj"
                font.pointSize: 18
                font.weight: Font.DemiBold
                color: darkgrey
                text: "Action ID"
                width: parent.width
                height: 100
                Layout.bottomMargin: 20
            }

            Text {
                id: mapTextOverlay

                font.pointSize: 12
                color: grey
                horizontalAlignment: Text.AlignHCenter
                width: parent.width
                text: "When the controller is running, priority levels and tasks will appear here."
                font.weight: Font.DemiBold
                visible: actionColumnView.children.length === 0
            }

            ColumnLayout {
                id: actionColumnView
                spacing: 25
                objectName: "actionColumnViewObj"

            }
        }

    }

}
