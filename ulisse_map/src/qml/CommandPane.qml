import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import QtQuick.Controls.Styles 1.4
//import QtQuick.Dialogs 2.0


Pane {
    id: commandRect
    Layout.alignment: Qt.AlignCenter
    Layout.preferredWidth: parent.width - panesMargin
    //Layout.preferredHeight: buttonsColumn.height
    Layout.bottomMargin: 10
    Material.elevation: myElevation

    ColumnLayout {
        id: buttonsColumn
        anchors.verticalCenter: parent.verticalCenter
        anchors.horizontalCenter: parent.horizontalCenter
        width: parent.width
        spacing: 0

        Label {
            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
            font.pointSize: 12
            font.weight: Font.DemiBold
            bottomPadding: 10
            color: 'seagreen'
            text: "Commands"
        }


        Button {
            text: "Halt"
            onClicked: cmdWrapper.sendHaltCommand()
        }

        RowLayout {
            Layout.fillWidth: true

            Button {
                id: holdButton
                text: "Hold Position"

                onClicked: {
                    if(holdRadiusText.text !== ''){
                        cmdWrapper.sendHoldCommand()
                    } else {
                        acceptRadDialog.open();
                    }
                }

            }

            Rectangle {
                id: holdSpacer
                width: buttonsColumn.width - holdButton.width - holdRadiusText.width
                height: parent.height
                anchors.left: holdButton.right
            }


            TextField {
                id: holdRadiusText
                Layout.maximumWidth: 40
                Layout.fillWidth: true
                anchors.left: holdSpacer.right
                font.pointSize: 10
                placeholderText: "Radius"
                selectByMouse: true

                validator: DoubleValidator {
                    bottom: 0.0;
                    top: 50.0;
                    decimals: 1;
                    notation: DoubleValidator.StandardNotation
                }
            }
        }

        RowLayout {
            Layout.fillWidth: true

            Button {
                id: moveToMarkButton
                text: "Move To Marker"

                onClicked: {
                    if(moveToMarkText.text !== ''){
                        cmdWrapper.sendLatLongCommand(marker_coords)
                    } else {
                        acceptRadDialog.open();
                    }
                }

            }

            Rectangle {
                id: moveToMarkSpacer
                width: buttonsColumn.width - moveToMarkButton.width - moveToMarkText.width
                height: parent.height
                anchors.left: moveToMarkButton.right
            }


            TextField {
                id: moveToMarkText
                Layout.maximumWidth: 40
                Layout.fillWidth: true
                anchors.left: moveToMarkSpacer.right
                font.pointSize: 10
                placeholderText: "Radius"
                selectByMouse: true

                //validator: RegExpValidator { regExp: /[0-9]{5}/ }
                validator: DoubleValidator {
                    bottom: 0.0;
                    top: 50.0;
                    decimals: 1;
                    notation: DoubleValidator.StandardNotation
                }
            }
        }

        RowLayout {
            Layout.fillWidth: true

            Button {
                id: speedHeadButton
                text: "Speed-Heading"

                onClicked: {
                    if(speedText.text !== '' && headingText.text !== ''){
                        cmdWrapper.sendLatLongCommand(marker_coords)
                    } else {
                        speedHeadingDialog.open();
                    }
                }

            }

            Rectangle {
                id: speedHeadSpacer
                width: buttonsColumn.width - speedHeadButton.width - speedText.width - headingText.width
                height: parent.height
                anchors.left: speedHeadButton.right
            }


            TextField {
                id: speedText
                Layout.maximumWidth: 40
                Layout.fillWidth: true
                anchors.left: speedHeadSpacer.right
                font.pointSize: 10
                placeholderText: "S"
                selectByMouse: true

                validator: DoubleValidator {
                    bottom: -5.0;
                    top: +5.0;
                    decimals: 1;
                    notation: DoubleValidator.StandardNotation
                }
            }

            TextField {
                id: headingText
                Layout.maximumWidth: 40
                Layout.fillWidth: true
                anchors.left: speedText.right
                font.pointSize: 10
                placeholderText: "H°"
                selectByMouse: true

                validator: IntValidator {
                    bottom: 0;
                    top: 360;

                }
            }
        }
    }

}
