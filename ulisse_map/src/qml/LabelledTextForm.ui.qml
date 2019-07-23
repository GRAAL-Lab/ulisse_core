import QtQuick 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1

ColumnLayout {
    property alias label: title.text
    property alias labelColor: title.color
    property alias text: label_data.text
    property alias textColor: label_data.color
    property alias textBoldness: label_data.font.weight
    property alias labelMouseArea: labelMouseArea

    Layout.preferredHeight: title.contentHeight + label_data.contentHeight + 5
    spacing: 0

    Label {
        id: title
        width: parent.width
        color: 'cadetblue'
        leftPadding: 5.0
        bottomPadding: -4
        font.pointSize: 9
        font.weight: Font.DemiBold
        //text: "Coordinates"
    }

    Label {
        id: label_data
        width: parent.width
        //color: 'lightgray'
        leftPadding: 5.0
        font.pointSize: 10.5

        MouseArea {
            id: labelMouseArea
            anchors.fill: parent
        }
    }
}
