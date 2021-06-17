import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import "."

Rectangle {

    property var titlesize: 15
    property var labelsize: 13
    color: Material.background

    ColumnLayout {
        id: utilitiesColumnView

        anchors.fill: parent

        Pane {
            id: buttonsPane
            Layout.fillWidth: true

            ColumnLayout {
                id: buttonsColumn
                width: parent.width
                Layout.fillHeight: true
                Layout.fillWidth: true

                Button {
                    id: kclReconfButton
                    text: "  Reload KCL Conf  "
                    Layout.alignment: Qt.AlignCenter
                    highlighted: true
                    Material.background: pressed ? orange : mainColor

                    onClicked: {
                        cmdWrapper.reloadKCLConf()
                    }
                }

                Button {
                    id: dclReconfButton
                    text: "  Reload DCL Conf  "
                    Layout.alignment: Qt.AlignCenter
                    highlighted: true
                    Material.background: pressed ? orange : mainColor

                    onClicked: {
                        cmdWrapper.reloadDCLConf()
                    }
                }

                Button {
                    id: navFilterReconfButton
                    text: "  Reload NavFilter Conf  "
                    Layout.alignment: Qt.AlignCenter
                    highlighted: true
                    Material.background: pressed ? orange : mainColor

                    onClicked: {
                        cmdWrapper.reloadNavFilterConf()
                    }
                }
            }
        }
    }
}
