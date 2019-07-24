import QtQuick 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import "."

RowLayout {
    property alias label: title.text
    property int lsize: 11
    property int tsize: 9
    property alias labelColor: title.color
    property alias text: label_data.text
    property alias textColor: label_data.color
    property alias textBoldness: label_data.font.weight
    property alias labelMouseArea: labelMouseArea
    //Layout.alignment: Qt.AlignHCenter
    spacing: 1

    Label {

        id: title
        horizontalAlignment: Qt.AlignRight
        font.pointSize: lsize
        color: 'cadetblue'
        Layout.columnSpan: 1
        font.weight: Font.DemiBold
    }

    Label {

        id: label_data
        horizontalAlignment: Qt.AlignRight
        Layout.columnSpan: 2
        font.pointSize: tsize
        leftPadding: 5
        //Layout.fillWidth: true
        MouseArea {
            id: labelMouseArea
            anchors.fill: parent
        }
    }
}
