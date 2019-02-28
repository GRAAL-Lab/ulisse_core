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
        console.log(mapChooser.displayText)
        settings.nextMapProvider = mapChooser.displayText
        console.log("Saved map: %1").arg(settings.nextMapProvider)
        close()
        stackViewContainer.forceActiveFocus()
    }
    onRejected: {
        styleBox.currentIndex = styleBox.styleIndex
        mapChooser.currentIndex = mapChooser.mapIndex
        close()
        stackViewContainer.forceActiveFocus()
    }

    contentItem: ColumnLayout {
        id: settingsColumn
        spacing: 20

        RowLayout {
            spacing: 10
            width: parent.width

            Label {
                text: "Speed/Heading Timeout (s):"
            }

            TextField {
                id: speedHeadingTimeout
                objectName: "speedHeadingTimeout"
                Layout.preferredWidth: 45
                Layout.fillWidth: true
                font.pointSize: 11
                text: "120"
                placeholderText: "Timeout"
                selectByMouse: true

                validator: IntValidator {
                    bottom: 0.0;
                    top: 5000.0;
                }
            }
        }

        /*RowLayout {
            spacing: 10

            Label {
                text: "Map Provider:"
            }
            ComboBox {
                id: mapChooser
                property int mapIndex: -1
                model: ["osm", "esri"]
                Component.onCompleted: {
                    mapIndex = find(settings.mapProvider, Qt.MatchFixedString)
                    if (mapIndex !== -1)
                        currentIndex = mapIndex
                }
                Layout.fillWidth: true
            }
        }*/

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
            opacity: mapChooser.currentIndex !== mapChooser.mapIndex ? 1.0 : 0.0
            horizontalAlignment: Label.AlignHCenter
            verticalAlignment: Label.AlignVCenter
            Layout.fillWidth: true
            Layout.fillHeight: true
        }*/
    }
}

