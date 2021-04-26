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
    /*property int h: 15
    property int w: 200*/
    //Layout.alignment: Qt.AlignHCenter
    spacing: 1
    Rectangle {
        //width: w
        //height: h
        color: "#00000000"

        Text {
            id: title
            horizontalAlignment: Text.AlignHCenter
            font.pointSize: lsize
            color: 'cadetblue'
            clip: true
            Layout.fillWidth: false
            Layout.columnSpan: 1
            font.weight: Font.DemiBold
        }
    }

    Rectangle {
        //width: 200
        //height: h
        color: "#00000000"

        Text {

            id: label_data
            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
            Layout.fillWidth: true
            //            horizontalAlignment: Text.Text.AlignHCenter
            Layout.columnSpan: 1
            font.pointSize: tsize
            //Layout.fillWidth: true
            MouseArea {
                id: labelMouseArea
                anchors.fill: parent
            }
        }
    }
}

