import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import "."


RowLayout {

    property var priorityID: "Priority Level"
    property var taskIDs: ["Task_1", "Task_2"]

    id: priorityLevelData
    objectName: "priorityLevelDataObj"

    spacing: 15

    Label {
        //Layout.fillHeight: true
        //Layout.alignment: Qt.AlignHCenter
        wrapMode: Label.WordWrap
        font.pointSize: 16
        font.weight: Font.DemiBold
        color: 'grey'
        text: priorityID
        Layout.preferredWidth: 100
    }

    Rectangle{
        Layout.fillHeight: true
        width: 1
        color: 'darkgrey'
    }

    ColumnLayout{

        spacing: 15

        Repeater {
            model: taskIDs

            TaskData{
                taskName: modelData
            }
        }
    }

}


