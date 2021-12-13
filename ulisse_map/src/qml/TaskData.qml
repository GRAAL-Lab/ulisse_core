import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import "."

Item {
    Layout.preferredHeight: taskDataColumn.implicitHeight
    Layout.fillWidth: true
    property var labelsize: 12
    property var textsize: 11

    property var taskName: "Undefined"
    property var reference: " "
    property var internalAct: " "
    property var externalAct: " "

    ColumnLayout {
        id: taskDataColumn
        width: parent.width
        Layout.fillHeight: true

        Label {
            Layout.fillHeight: true
            //Layout.alignment: Qt.AlignHCenter
            font.pointSize: 14
            font.weight: Font.DemiBold
            color: Material.color(Material.Blue, Material.Shade700)
            text: taskName
        }

        LabelledText {
            Layout.alignment: Qt.AlignHCenter
            labelColor: 'dodgerblue'
            textColor: 'grey'
            tsize: textsize
            label: qsTr("Reference")
            text: reference


        }

        LabelledText {
            //Layout.fillHeight: true
            Layout.alignment: Qt.AlignHCenter
            labelColor: 'dodgerblue'
            textColor: 'grey'
            tsize: textsize
            label: qsTr("Internal Act.")
            text: internalAct
        }

        LabelledText {
            Layout.alignment: Qt.AlignHCenter
            labelColor: 'dodgerblue'
            tsize: textsize
            textColor: 'grey'
            label: qsTr("External Act.")
            text: externalAct

        }
    }
}
