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
        width: 200
        horizontalAlignment: Text.AlignLeft
        font.pointSize: lsize
        color: 'cadetblue'
        Layout.fillWidth: false
        Layout.columnSpan: 1
        font.weight: Font.DemiBold
    }

    Label {

        id: label_data
        Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
        Layout.fillWidth: true
        horizontalAlignment: Text.AlignLeft
        Layout.columnSpan: 1
        font.pointSize: tsize
        leftPadding: 5
        //Layout.fillWidth: true
        MouseArea {
            id: labelMouseArea
            anchors.fill: parent
        }
    }
}

/*##^##
Designer {
    D{i:0;width:400}
}
##^##*/

