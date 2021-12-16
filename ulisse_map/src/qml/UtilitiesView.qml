import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import QtQuick.Dialogs 1.2
import "."

Rectangle {

    property bool recording: false
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

                Label {
                    Layout.fillHeight: true
                    Layout.alignment: Qt.AlignHCenter
                    font.pointSize: 18
                    font.weight: Font.DemiBold
                    color: blue
                    text: "Reload Configurations"
                }

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

                ToolSeparator {
                    orientation: Qt.Horizontal
                    Layout.fillWidth: true

                    contentItem: Rectangle {
                        implicitHeight: 1
                        color: "#c3c3c3"
                    }
                }

                Label {
                    Layout.fillHeight: true
                    Layout.alignment: Qt.AlignHCenter
                    font.pointSize: 18
                    font.weight: Font.DemiBold
                    color: blue
                    text: "Reset ROS"
                }

                Button {
                    id: reloadPubsAndSubsButton
                    text: "  Reload Pub/Sub "
                    Layout.alignment: Qt.AlignCenter
                    highlighted: true
                    Material.background: pressed ? orange : mainColor

                    onClicked: {
                        cmdWrapper.resetPublishersAndSubscribers()
                        fbkUpdater.resetPublishersAndSubscribers()
                        taskdataUpdater.resetPublishersAndSubscribers()
                    }
                }

                ToolSeparator {
                    orientation: Qt.Horizontal
                    Layout.fillWidth: true

                    contentItem: Rectangle {
                        implicitHeight: 1
                        color: "#c3c3c3"
                    }
                }

                Label {
                    Layout.fillHeight: true
                    Layout.alignment: Qt.AlignHCenter
                    font.pointSize: 18
                    font.weight: Font.DemiBold
                    color: blue
                    text: "Record Bag"
                }

                RowLayout {
                    spacing: 10

                    Layout.alignment: Qt.AlignHCenter

                    Label {
                        id: saveFolderLabel
                        text: qsTr("Save folder: ")
                        font.pointSize: 11
                        font.weight: Font.DemiBold
                        color: recording ? grey : 'black'
                    }

                    TextField {
                        id: saveFolderPath
                        placeholderText: "Enter save folder path..."
                        text: settings.bagSaveFolder
                        Layout.preferredWidth: 350
                        font.pointSize: 11
                        selectByMouse: true
                        enabled: !recording

                        onTextChanged: {
                            settings.bagSaveFolder = saveFolderPath.text
                        }
                    }

                    /*Button {
                        id: fillHomePathButton
                        text: "Home Path"
                        font.pointSize: 9
                        highlighted: true
                        Material.background: grey

                        onClicked: {
                            saveFolderPath.text = home_dir
                        }
                    }*/

                    Button {
                        id: browsePathButton
                        text: "Browse..."
                        font.pointSize: 9
                        highlighted: true
                        Material.background: grey

                        onClicked: {
                            browseBagDirDialog.open()
                        }

                        FileDialog {
                            id: browseBagDirDialog
                            title: "Please choose a folder"
                            folder: shortcuts.home
                            selectFolder: true

                            onAccepted: {
                                var path = browseBagDirDialog.fileUrl.toString()
                                // remove prefixed "file://"
                                path = path.replace(/^(file:\/{2})/, "")
                                // unescape html codes like '%23' for '#'
                                var cleanPath = decodeURIComponent(path)
                                saveFolderPath.text = cleanPath
                            }
                        }
                    }
                }

                RowLayout {
                    spacing: 10

                    Layout.alignment: Qt.AlignHCenter

                    Label {
                        id: bagInfoLabel
                        text: qsTr("Bag Info: ")
                        font.pointSize: 11
                        font.weight: Font.DemiBold
                        color: recording ? grey : 'black'
                    }

                    TextArea {
                        id: bagInfoText
                        Layout.preferredWidth: 350
                        placeholderText: "Enter bag description (optional)"
                        wrapMode: TextEdit.WordWrap
                        Material.accent: grey
                        font.pointSize: 11
                        selectByMouse: true
                        enabled: !recording

                    }
                }

                Button {
                    id: recordBagButton
                    text: recording ? "  Stop Recording  " : "  Start Recording  "
                    Layout.alignment: Qt.AlignCenter
                    highlighted: true
                    Material.background: pressed ? orange : (recording ? red : mainColor)

                    onClicked: {
                        if (!recording){
                            recording = cmdWrapper.sendRosbagRecordCommand(true, saveFolderPath.text, bagInfoText.text)
                        } else {
                            recording = cmdWrapper.sendRosbagRecordCommand(false)
                        }
                    }
                }

                Label {
                    id: bagRecordInProgress
                    text: "Rosbag recording in progress..."
                    color: red
                    opacity: recording ? 1.0 : 0.0
                    font.weight: Font.DemiBold
                    horizontalAlignment: Label.AlignHCenter
                    verticalAlignment: Label.AlignVCenter
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                }
            }
        }
    }
}
