import QtQuick 2.5
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.1
import "."

Dialog {

    property alias mapCacheDirText: mapCacheDirectory.text

    modal: true
    focus: true
    title: "Settings"

    standardButtons: Dialog.Ok | Dialog.Cancel
    onAccepted: {
        /*if (mapTypeBox.displayText != futureMapPlugin) {
            futureMapPlugin = mapTypeBox.displayText
            toast.show("Changes will take effect on restart...", 2000)
        }*/

        if (mapCacheDirectory.displayText != settings.esriMapCacheDir) {
            settings.esriMapCacheDir = mapCacheDirectory.displayText
            toast.show("Changes will take effect on restart...", 2000)
        }

        //settings.theme = styleBox.displayText

        close()
        stackViewContainer.forceActiveFocus()
    }
    onRejected: {
        //styleBox.currentIndex = styleBox.styleIndex
        //mapTypeBox.currentIndex = mapTypeBox.mapTypeIndex
        mapCacheDirectory.text = settings.esriMapCacheDir

        close()
        stackViewContainer.forceActiveFocus()
    }

    contentItem: ColumnLayout {
        id: settingsColumn
        spacing: 20

        RowLayout {
            id: mapCacheSetting
            spacing: 10
            enabled: settings.mapPluginType === "esri" ? true : false

            Label {
                text: "Map Cache Directory:"
            }

            TextField {
                property bool cacheDirChanged: false

                id: mapCacheDirectory
                Layout.preferredWidth: 45
                Layout.fillWidth: true
                font.pointSize: 10
                text: settings.esriMapCacheDir
                placeholderText: "Folder path"
                selectByMouse: true

                onTextChanged: {
                    if (text != settings.esriMapCacheDir) {
                        cacheDirChanged = true
                    } else {
                        cacheDirChanged = false
                    }
                }
            }

            Button {
                id: selectMapCache
                text: "Browse"
                font.pointSize: 10

                onClicked: {
                    browseCacheDirDialog.open()
                }
            }
        }

        RowLayout {
            id: showStatusOverlaySetting
            spacing: 10

            Label {
                text: "Show Overlay on Map:"
            }
            CheckBox {
                id: statusOverlayBox
                text: "Show Overlay"
                //Material.accent: orange
                checked: true

                onClicked: {
                    settings.showStatusOverlay = !settings.showStatusOverlay;
                }
            }
        }


        RowLayout {
            id: showCentroidSetting
            spacing: 10

            Label {
                text: "Show Centroid on Map:"
            }
            CheckBox {
                id: showCentroidBox
                text: "Show Centroid"
                font.pointSize: 10
                checkState: settings.showCentroid ? Qt.Checked : Qt.Unchecked

                onClicked: {
                    settings.showCentroid = !settings.showCentroid;
                }
            }
        }

        RowLayout {
            id: showObstacleIDSetting
            spacing: 10

            Label {
                text: "Show obstacles ID on map:"
            }
            CheckBox {
                id: showObstacleIDBox
                text: "Show IDs"
                font.pointSize: 10
                checkState: settings.showObstacleID ? Qt.Checked : Qt.Unchecked

                onClicked: {
                    settings.showObstacleID = !settings.showObstacleID;
                }
            }
        }

        /*RowLayout {
            id: appStyleSetting
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
        }*/

        Label {
            id: restartText
            text: "Restart required!"
            color: "#e41e25"
            opacity: mapCacheDirectory.cacheDirChanged ? 1.0 : 0.0
            font.weight: Font.DemiBold
            horizontalAlignment: Label.AlignHCenter
            verticalAlignment: Label.AlignVCenter
            Layout.fillWidth: true
            Layout.fillHeight: true
        }
    }
}
