import QtQuick 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1

ColumnLayout {
    property alias label: title.text
    property int lsize: 12
    property int tsize: 9
    property alias labelColor: title.color
    property alias text: label_data.text
    property alias textColor: label_data.color
    property alias textBoldness: label_data.font.weight
    property alias labelMouseArea: labelMouseArea

    Layout.preferredHeight: title.contentHeight + label_data.contentHeight + 5
    spacing: 0

    Label {
        Layout.alignment: Qt.AlignHCenter
        id: title
        font.pointSize : lsize
        width: parent.width
        color: 'cadetblue'
        leftPadding: 5.0
        bottomPadding: -4
        font.weight: Font.DemiBold
        //text: "Coordinates"
    }

    Label {
        Layout.alignment: Qt.AlignHCenter
        id: label_data
        font.pointSize: tsize
        width: parent.width
        //color: 'lightgray'
        leftPadding: 5.0

        MouseArea {
            id: labelMouseArea
            anchors.fill: parent
        }
    }
}
