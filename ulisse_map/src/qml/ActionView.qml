import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import "."

Rectangle {

    id: actionView
    objectName: "actionViewObj"

    property var titlesize: 15
    property var labelsize: 13
    color: Material.background

    Pane {
        anchors.fill: parent
        anchors.margins: 20




        ColumnLayout {
            id: actionColumnView
            spacing: 25
            objectName: "actionColumnViewObj"

            Label {
                objectName: "actionLabelObj"
                font.pointSize: 18
                font.weight: Font.DemiBold
                color: darkgrey
                text: "Action ID"
                Layout.preferredWidth: 100
            }


        }

        /*function generatePriorityLevel(plName, tasksName){

        var component = Qt.createComponent("PriorityLevelData.qml")
        var componentObject

        function finishCreation() {
            componentObject = component.createObject( taskColumnView, {priorityID: plName, tasks: tasksName} );
        }

        if (component.status === Component.Ready) {
            finishCreation()
        } else {
            component.statusChanged.connect( finishCreation );
        }

        console.log("createPriorityLevel: ".concat(plName));
        return "PL View Created"
    }
*/
    }

}
