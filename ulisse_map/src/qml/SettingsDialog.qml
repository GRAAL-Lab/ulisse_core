import QtQuick 2.5
import QtQuick.Layouts 1.1
import QtQuick.Controls 2.5
import QtQuick.Controls.Material 2.5
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

        settings.obstacleTimeout = obstacleTimeoutSeconds.value * 1
        console.log("Setting obstacle timeout to " + settings.obstacleTimeout + " s")

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
        spacing: 15

        RowLayout {
            id: mapTypeSetting
            spacing: 10
            enabled: settings.mapPluginType === "esri" ? true : false

            Label {
                text: "Map Type:"
            }

            ComboBox {
                id: mapTypeComboBox

                property bool changed: false

                model: ["A", "B", "C"]/*ListModel {
                    id: model
                    ListElement { text: "MapType.StreetMap";         color: "Brown" } // A street map.
                    ListElement { text: "MapType.SatelliteMapDay";   color: "Brown" } // A map with day-time satellite imagery.
                    ListElement { text: "MapType.SatelliteMapNight"; color: "Brown" } // A map with night-time satellite imagery.
                    ListElement { text: "MapType.TerrainMap";        color: "Brown" } // A terrain map.
                    ListElement { text: "MapType.HybridMap";         color: "Brown" } // A map with satellite imagery and street information.
                    ListElement { text: "MapType.GrayStreetMap";     color: "Brown" } // A gray-shaded street map.
                }*/

                onAccepted: {
                    if (currentText !== settings.esriMapType) {
                        changed = true
                    } else {
                        changed = false
                    }
                }

                Component.onCompleted: {
                    var types = []

                    /*console.log("map.supportedMapTypes")
                    for(var i=0; i< map.supportedMapTypes.length; i++){
                        console.log(map.supportedMapTypes[i].description)
                    }*/
                }


            }
        }

        //        MapType.StreetMap - A street map.
        //        MapType.SatelliteMapDay - A map with day-time satellite imagery.
        //        MapType.SatelliteMapNight - A map with night-time satellite imagery.
        //        MapType.TerrainMap - A terrain map.
        //        MapType.HybridMap - A map with satellite imagery and street information.
        //        MapType.GrayStreetMap - A gray-shaded street map.

        RowLayout {
            id: mapCacheSetting
            spacing: 10
            enabled: settings.mapPluginType === "esri" ? true : false

            Label {
                text: "Map Cache Directory:"
            }

            TextField {
                property bool changed: false

                id: mapCacheDirectory
                Layout.preferredWidth: 45
                Layout.fillWidth: true
                font.pointSize: 10
                text: settings.esriMapCacheDir
                placeholderText: "Folder path"
                selectByMouse: true

                onTextChanged: {
                    if (text != settings.esriMapCacheDir) {
                        changed = true
                    } else {
                        changed = false
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

        RowLayout {
            id: osbtacleTimeoutSetting
            spacing: 10

            Label {
                text: "Osbtacle deletion timeout (s):"
            }

            SpinBox {
                id: obstacleTimeoutSeconds
                from: 1
                to: 1000
                stepSize: 1
                Layout.maximumWidth: 150
                Layout.fillWidth: true
                font.pointSize: 10
                editable: true
                inputMethodHints: Qt.ImhDigitsOnly //Only digits are allowed.

                Component.onCompleted: {
                    value = settings.obstacleTimeout
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
            opacity: (mapCacheDirectory.cacheDirChanged || mapTypeSetting.changed) ? 1.0 : 0.0
            font.weight: Font.DemiBold
            horizontalAlignment: Label.AlignHCenter
            verticalAlignment: Label.AlignVCenter
            Layout.fillWidth: true
            Layout.fillHeight: true
        }
    }

}
