import QtQuick 2.5
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.1
import QtGraphicalEffects 1.0

Dialog {
    id: dialog
    property alias dialog: dialog

    modal: true
    focus: true
    //title: "Help"

    standardButtons: Dialog.Ok
    onAccepted: {
        close()
    }

    contentItem: ColumnLayout {
        id: helpColumn
        spacing: 10
        width: parent.width
        height: parent.height

        Label {
            id: shortcutsTitle
            text: "Shortcuts"
            font.pointSize: 12
            font.bold: true
            verticalAlignment: Text.AlignBottom
            horizontalAlignment: Label.AlignHCenter
            //verticalAlignment: Label.AlignVCenter
            Layout.fillWidth: true
            //Layout.fillHeight: true
            //Layout.bottomMargin: - helpColumn.spacing
        }

        Text {
            //width: parent.width - 20
            font.pointSize: 11
            //readOnly: true
            text: "<b>Spacebar</b>: Send a Halt Command"
            color: Material.foreground
            wrapMode: Text.Wrap
            anchors.top: shortcutsTitle.bottom
            anchors.topMargin: 5

        }

        Label {
            id: aboutTitle
            text: "About"
            verticalAlignment: Text.AlignBottom
            font.pointSize: 12
            font.bold: true
            horizontalAlignment: Label.AlignHCenter
            //verticalAlignment: Label.AlignVCenter
            Layout.fillWidth: true
            //Layout.fillHeight: true
            //Layout.bottomMargin: - helpColumn.spacing
        }

        TextArea {
            id: aboutField
            font.pointSize: 11
            width: parent.width
            readOnly: true
            text: "Lorem ipsum dolor sit amet, consectetur adipiscing elit. Proin viverra in nibh ac auctor. Vestibulum id fermentum elit, sed mollis libero. Mauris sed vehicula tortor, et tempor erat. Maecenas orci est, dignissim non mattis sit amet, tincidunt at risus."
            wrapMode: Text.WordWrap
            anchors.top: aboutTitle.bottom
            anchors.topMargin: 5

        }

        Label {
            text: "Developed by GRAAL Lab (2019)"
            color: "gray"
            horizontalAlignment: Label.AlignHCenter
            //verticalAlignment: Label.AlignVCenter
            Layout.fillWidth: true
            //Layout.fillHeight: true
            anchors.bottom: graalLogo.top
            anchors.bottomMargin: 10
        }

        Image {
            id: graalLogo
            height: 20
            fillMode: Image.PreserveAspectFit
            source: 'qrc:/images/graal_logo.png'
            Layout.fillWidth: true
            MouseArea {
                anchors.fill: parent

                onClicked:  {
                    eastereggDialog.open();
                }
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
        }
    }
}
