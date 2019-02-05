import QtQuick 2.5
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.1

Dialog {

    modal: true
    focus: true
    title: "Settings"

    standardButtons: Dialog.Ok | Dialog.Cancel
    onAccepted: {
        settings.theme = styleBox.displayText
        close()
    }
    onRejected: {
        styleBox.currentIndex = styleBox.styleIndex
        close()
    }

    contentItem: ColumnLayout {
        id: settingsColumn
        spacing: 20

        RowLayout {
            spacing: 10
            width: parent.width
            //horizontalAlignment: Label.AlignHCenter

            Label {
                text: "Speed/Heading Timeout (s):"
            }

            TextField {
                id: speedHeadingTimeout
                objectName: "speedHeadingTimeout"
                Layout.preferredWidth: 45
                Layout.fillWidth: true
                font.pointSize: 10
                text: "120"
                placeholderText: "Timeout"
                selectByMouse: true

                validator: IntValidator {
                    bottom: 0.0;
                    top: 5000.0;
                }
            }
        }

        RowLayout {
            spacing: 10

            Label {
                text: "Application Theme:"
            }

            ComboBox {
                id: styleBox
                property int styleIndex: -1
                model: ["Light", "Dark"]
                Component.onCompleted: {
                    styleIndex = find(settings.theme, Qt.MatchFixedString)
                    if (styleIndex !== -1)
                        currentIndex = styleIndex
                }
                Layout.fillWidth: true
            }
        }

        /*Label {
            text: "Restart required"
            color: "#e41e25"
            opacity: styleBox.currentIndex !== styleBox.styleIndex ? 1.0 : 0.0
            horizontalAlignment: Label.AlignHCenter
            verticalAlignment: Label.AlignVCenter
            Layout.fillWidth: true
            Layout.fillHeight: true
        }*/
    }
}

