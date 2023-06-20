import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import "."

Item {
    id: taskData
    Layout.preferredHeight: taskDataColumn.implicitHeight
    Layout.fillWidth: true
    property real labelsize: 12
    property real textsize: 11

    property string taskName: "Undefined"
    property string reference: " "
    property string internalAct: " "
    property string externalAct: " "
    property bool enabled: false

    ColumnLayout {
        id: taskDataColumn
        width: parent.width
        Layout.fillHeight: true

        RowLayout {
            //spacing: 5
            Label {
                Layout.fillHeight: true
                //Layout.alignment: Qt.AlignHCenter
                font.pointSize: 14
                font.weight: Font.DemiBold
                color: Material.color(Material.Blue, Material.Shade700)
                text: taskName//.replace(/_/g, " ");

            }

            Rectangle {
                width: taskEnableInfo.contentWidth + 6
                height: taskEnableInfo.contentHeight + 6
                border.color: taskData.enabled ? "green" : "red"
                border.width: 1
                radius: 5

                Text {
                    id: taskEnableInfo
                    verticalAlignment: Text.AlignVCenter
                    horizontalAlignment: Text.AlignHCenter
                    anchors.fill: parent
                    font.pointSize: 8
                    color: taskData.enabled ? "green" : "red"
                    text:  taskData.enabled ? qsTr("Enabled") : qsTr("Disabled")
                }
            }
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
