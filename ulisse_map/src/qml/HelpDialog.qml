import QtQuick 2.5
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.1
import QtGraphicalEffects 1.0
import "."

Dialog {
    id: dialog
    property alias dialog: dialog

    modal: true
    focus: true

    standardButtons: Dialog.Ok
    onAccepted: {
        close()
        stackViewContainer.forceActiveFocus()
    }

    contentItem: ColumnLayout {
        id: helpColumn
        spacing: 10
        width: parent.width
        height: parent.height

        Label {
            id: helpTitle
            text: "Help"
            font.pointSize: 14
            font.bold: true
            verticalAlignment: Text.AlignBottom
            horizontalAlignment: Label.AlignHCenter
            Layout.fillWidth: true
        }

        Text {
            font.pointSize: 11
            text: "<b>Offline maps</b>: to use the offline maps, select the OpenStreetMaps plugin in the settings
                and then the map type \"Street maps in daylight view\".<br>"
            lineHeight: 1.5
            color: Material.foreground
            wrapMode: Text.Wrap
            Layout.alignment: Qt.AlignLeft | Qt.AlignVCenter
            Layout.maximumWidth: parent.width - 30
        }


        Label {
            id: shortcutsTitle
            text: "Shortcuts"
            font.pointSize: 12
            font.bold: true
            verticalAlignment: Text.AlignBottom
            horizontalAlignment: Label.AlignHCenter
            Layout.fillWidth: true
            Layout.topMargin: 20
        }

        Text {
            font.pointSize: 11
            lineHeight: 1.5
            text: "<b>• Return</b>: Send a Halt Command.<br>
                   <b>• Fast Copy/Paste</b>: Most of the numerical values in the interface can be copied to clipboard just
                    by left clicking on the number."
            color: Material.foreground
            wrapMode: Text.Wrap
            Layout.alignment: Qt.AlignLeft | Qt.AlignVCenter
            Layout.maximumWidth: parent.width - 30
        }

        Label {
            id: aboutTitle
            text: "About"
            verticalAlignment: Text.AlignBottom
            font.pointSize: 12
            font.bold: true
            horizontalAlignment: Label.AlignHCenter
            Layout.fillWidth: true
            Layout.topMargin: 20
        }

        TextArea {
            id: aboutField
            font.pointSize: 11
            width: parent.width
            readOnly: true
            text: "This is a front-end interface to control the Ulisse catamaran. "
            wrapMode: Text.WordWrap
            Layout.alignment: Qt.AlignLeft | Qt.AlignVCenter
        }

        Label {
            text: "Developed by GRAAL Lab (2023)<br>Mantainer: francesco.wanderlingh@unige.it"
            horizontalAlignment: Text.AlignHCenter
            color: "grey"
            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
            Layout.fillWidth: true
        }

        Image {
            id: graalLogo
            Layout.minimumHeight: 20
            fillMode: Image.PreserveAspectFit
            source: 'qrc:/images/graal_logo.png'
            Layout.fillWidth: true
            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter

            MouseArea {
                anchors.fill: parent
                onClicked:  {
                    eastereggDialog.open();
                }
            }

            ModalPopup {
                id: eastereggDialog
                title: "Interfaccia bella assai"
                contentItem: Text {
                    id: theText
                    width: parent.width
                    color: "#ff7644"
                    text: "by <b>wanderfra</b>"
                    font.bold: true
                    verticalAlignment: Text.AlignVCenter
                    horizontalAlignment: Text.AlignHCenter
                    styleColor: "#ffead1"
                    style: Text.Outline
                }
                //Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                anchors.centerIn: parent
            }
        }
    }
}
