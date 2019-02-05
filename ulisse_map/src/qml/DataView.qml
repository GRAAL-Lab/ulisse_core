import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import "."

Rectangle {
    property real myElevation: 6
    property real panesMargin: 14

    Flow {
        width: parent.width

        Pane {
            id: batteryPane
            width: parent.width * 0.3
            Material.elevation: myElevation

            ColumnLayout {
                id: batteryData
                width: parent.width

                Label {
                    Layout.alignment: Qt.AlignHCenter
                    font.pointSize: 11
                    font.weight: Font.DemiBold
                    color: 'gray'
                    text: "Battery"
                }

                LabelledText {
                    id: batteryPercLeft
                    labelColor: 'tomato'
                    label: "Left"
                    textColor: 'gray'
                    text: "%1 \%".arg(fbkUpdater.battery_perc_L)
                }

                LabelledText {
                    id: batteryPercRight
                    labelColor: 'tomato'
                    label: "Right"
                    textColor: 'gray'
                    text: "%1 \%".arg(fbkUpdater.battery_perc_R)
                }
            }
        }
    }
}
