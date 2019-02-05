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
    property real panesWidth: 252
    property bool responsive: false
    color: Material.background

    function getPaneWidth(parentID){
        var itemsPerRow = 3
        var paneWidth = responsive ? panesWidth : parentID.width / itemsPerRow - panesMargin / 2
        return paneWidth;
    }

    Flow {
        id: flowView
        width: parent.width
        spacing: 10

        Pane {
            id: statusPane
            width: getPaneWidth(parent)
            Material.elevation: myElevation

            ColumnLayout {
                id: statusata
                width: parent.width

                Label {
                    Layout.alignment: Qt.AlignHCenter
                    font.pointSize: 11
                    font.weight: Font.DemiBold
                    color: 'gray'
                    text: "Status"
                }

                LabelledText {
                    //id: batteryPercRight
                    labelColor: 'dodgerblue'
                    label: "Vehicle State"
                    textColor: 'gray'
                    text: "Text"
                }

                LabelledText {
                    //id: batteryPercLeft
                    labelColor: 'dodgerblue'
                    label: "GPS time"
                    textColor: 'gray'
                    text: "Text"
                }

                LabelledText {
                    //id: batteryPercRight
                    labelColor: 'dodgerblue'
                    label: "GPS Pos"
                    textColor: 'gray'
                    text: "Text"
                }

                LabelledText {
                    //id: batteryPercRight
                    labelColor: 'dodgerblue'
                    label: "Filtered Pos"
                    textColor: 'gray'
                    text: "Text"
                }

                LabelledText {
                    //id: batteryPercRight
                    labelColor: 'dodgerblue'
                    label: "Heading"
                    textColor: 'gray'
                    text: "Text"
                }
            }
        }

        Pane {
            id: goalPane
            width: getPaneWidth(parent)
            Material.elevation: myElevation

            ColumnLayout {
                id: goalData
                width: parent.width

                Label {
                    Layout.alignment: Qt.AlignHCenter
                    font.pointSize: 11
                    font.weight: Font.DemiBold
                    color: 'gray'
                    text: "Goal"
                }

                LabelledText {
                    //id: batteryPercLeft
                    labelColor: 'seagreen'
                    label: "Goal Position"
                    textColor: 'gray'
                    text: "Text"
                }

                LabelledText {
                    //id: batteryPercRight
                    labelColor: 'seagreen'
                    label: "Goal Heading"
                    textColor: 'gray'
                    text: "Text"
                }
            }
        }

        Pane {
            id: ctrlPane
            width: getPaneWidth(parent)
            Material.elevation: myElevation

            ColumnLayout {
                id: ctrlData
                width: parent.width

                Label {
                    Layout.alignment: Qt.AlignHCenter
                    font.pointSize: 11
                    font.weight: Font.DemiBold
                    color: 'gray'
                    text: "Control"
                }

                LabelledText {
                    //id: batteryPercLeft
                    labelColor: 'orange'
                    label: "Desired Speed"
                    textColor: 'gray'
                    text: "Text"
                }

                LabelledText {
                    //id: batteryPercRight
                    labelColor: 'orange'
                    label: "Desired Jog"
                    textColor: 'gray'
                    text: "Text"
                }

                LabelledText {
                    //id: batteryPercRight
                    labelColor: 'orange'
                    label: "Motor Control Ref"
                    textColor: 'gray'
                    text: "Text"
                }
            }
        }

        Pane {
            id: batteryPane
            width: getPaneWidth(parent)
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
        /*
        Goal Pos:	0, 0
        Goal Distance:	0
        Goal Heading:	0
        Desired Speed:	0
        Desired Jog:	0
        Motor Map Out:	0, 0
        Motor Ctrl Ref:	0, 0
        */

    }
}
