import QtQuick 2.5
import QtQuick.Layouts 1.1
import QtQuick.Controls 2.5
import QtQuick.Controls.Material 2.5
import "."

Dialog {

    property alias mapCacheDirText: mapCacheDirectory.text
    property var mapTypesDescription: ({})
    modal: true
    focus: true
    title: "Settings"
    standardButtons: Dialog.Ok | Dialog.Cancel

    ListModel { id: myMapTypes }

    Timer {
        interval: 500; running: true; repeat: false
        onTriggered: {
            // The reason for creating this custom variable is a workaround for the fact
            // that not all map types have a meaningful description in the "esri" plugin.
            if (settings.mapPluginType === "esri"){
                for(var i=0; i< mapViewItem.map.supportedMapTypes.length; i++){
                    if (i === 1) {
                        myMapTypes.append({"description": "ArcGIS Satellite Imagery Map"})
                    } else {
                        myMapTypes.append({"description": mapViewItem.map.supportedMapTypes[i].description})
                    }
                }
            }
            mapTypeComboBox.currentIndex = settings.mapTypeIndex
        }
    }

    onAccepted: {
        settings.mapTypeIndex = mapTypeComboBox.currentIndex

        if (mapCacheDirectory.changed) {
            settings.esriMapCacheDir = mapCacheDirectory.displayText
            toast.show("Changes will take effect on restart...", 4000)
        }

        if (visualizerTimeoutSeconds.value != settings.visualizerTimeout) {
            settings.visualizerTimeout = visualizerTimeoutSeconds.value
            console.log("Setting obstacle timeout to " + settings.visualizerTimeout + " s")
        }

        close()
        stackViewContainer.forceActiveFocus()
    }
    onRejected: {
        mapCacheDirectory.text = settings.esriMapCacheDir

        close()
        stackViewContainer.forceActiveFocus()
    }

    contentItem: ColumnLayout {
        id: settingsColumn
        spacing: 3

        SettingsSectionLabel {
            text: "Map Plugin"
        }

        RowLayout {
            id: mapTypeSetting
            spacing: 10

            Label {
                text: "Map Type:"
            }

            ComboBox{
                id: mapTypeComboBox
                model: myMapTypes
                Layout.fillWidth: true
                textRole: "description"

                onCurrentIndexChanged: {
                    mapViewItem.map.activeMapType = mapViewItem.map.supportedMapTypes[currentIndex]
                }
            }
            // qml: ArcGIS Online World Street Map
            // qml:
            // qml: ArcGIS Online World Terrain Base
            // qml: ArcGIS Online World Topography
            // qml: This map presents land cover and detailed topographic maps for the United States.
            // qml: National Geographic World Map
            // qml: Thematic content providing a neutral background with minimal colors
            // qml: Natural Earth physical map for the world
            // qml: Portrays surface elevation as shaded relief
            // qml: This map is designed to be used as a basemap by marine GIS professionals and as a reference map by anyone interested in ocean data
            // qml: Thematic content providing a neutral background with minimal colors
            // qml: DeLorme’s topographic basemap is a seamless global data set that portrays transportation, hydrography, jurisdiction boundaries, and major geographic features
        }

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

        SettingsSectionLabel {
            text: "Catamaran Options"
        }

        RowLayout {
            id: showStatusOverlaySetting
            spacing: 10

            Label {
                text: "Show status overlay on Map:"
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

        SettingsSectionLabel {
            text: "Other Options"
        }

        RowLayout {
            id: showPolylineIDSetting
            spacing: 10

            Label {
                text: "Show polylines ID on map:"
            }
            CheckBox {
                id: showPolylineIDBox
                text: "Show IDs"
                font.pointSize: 10
                checkState: settings.showPolylineID ? Qt.Checked : Qt.Unchecked

                onClicked: {
                    settings.showPolylineID = !settings.showPolylineID;
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
                text: "Object visualizer deletion timeout (s):"
            }

            SpinBox {
                id: visualizerTimeoutSeconds
                from: 1
                to: 1000
                stepSize: 1
                Layout.maximumWidth: 150
                Layout.fillWidth: true
                font.pointSize: 10
                editable: true
                inputMethodHints: Qt.ImhDigitsOnly //Only digits are allowed.

                Component.onCompleted: {
                    value = settings.visualizerTimeout
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
            opacity: (mapCacheDirectory.changed) ? 1.0 : 0.0
            font.weight: Font.DemiBold
            horizontalAlignment: Label.AlignHCenter
            verticalAlignment: Label.AlignVCenter
            Layout.fillWidth: true
            Layout.fillHeight: true
        }
    }

}
