import QtQuick 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls 1.2

ColumnLayout {
    property alias label: title.text
    property alias labelColor: title.color
    property alias text: data.text
    property alias textColor: data.color

    //id: markerTextColumn
    width: parent.width
    Layout.preferredHeight: title.contentHeight + data.contentHeight + 10
    spacing: 0

    Label {
        id: title
        width: parent.width
        color: 'gray'
        leftPadding: 5.0
        bottomPadding: -5
        font.pointSize: 9
        font.weight: Font.DemiBold
        //text: "Coordinates"
    }

    Label {
        id: data
        width: parent.width
        color: 'lightgray'
        leftPadding: 5.0
        font.pointSize: 11
        //text: "Right click on map"
    }
}
