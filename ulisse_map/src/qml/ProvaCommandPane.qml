import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.2

CommandPaneForm {
    id:commandePane

    commandLabel.color: Material.color(Material.Green, Material.Shade700)

    haltButton.onClicked: {
                map.interruptPathIfActive()
                cmdWrapper.sendHaltCommand()
            }
    speedHeadButton.onClicked: {
                    if (speedText.text !== '' && headingText.text !== '') {
                        map.interruptPathIfActive()
                        cmdWrapper.sendSpeedHeadingCommand(speedText.text,
                                                           headingText.text)
                    } else {
                        speedHeadingDialog.open()
                    }
                }

    holdButton.onClicked: {
                    if (holdRadius.text !== '') {
                        map.interruptPathIfActive()
                        cmdWrapper.sendHoldCommand(parseFloat(holdRadius.text))
                    } else {
                        acceptRadDialog.open()
                    }
                }
    moveToButton.onClicked: {
                    if (moveToRadius.text !== '') {
                        map.interruptPathIfActive()
                        if (cmdWrapper.sendLatLongCommand(
                                    marker_coords,
                                    parseFloat(moveToRadius.text))) {
                            markerIcon.opacity = 0.2
                        }
                    } else {
                        acceptRadDialog.open()
                    }
                }
    moveToButton.enabled: {map.markerIconOpacity > 0 ? true : false}

    goToPreviousWp.onClicked: {
                    cmdWrapper.goToPreviousWaypoint()
                }

    goToPreviousWp.enabled: mapView.pathCurrentState === pathState.active ? true : false

    goToNextWp.onClicked: {
                    cmdWrapper.goToNextWaypoint()
                }

    goToNextWp.enabled: mapView.pathCurrentState === pathState.active ? true : false

    loopPathCB.onCheckStateChanged: {
                    if (checked === true) {
                        wpCommands.loopPath = true
                    } else {
                        wpCommands.loopPath = false
                    }
                }
    savePath.onClicked: {
                    savePathDialog.open()
                }

    savePath.enabled: (mapView.pathCurrentState === pathState.empty)
             | (mapView.pathCurrentState === pathState.creating) ? false : true

    loadPath.onClicked: {
                    loadPathDialog.open()
                }

    loadPath.enabled: (mapView.pathCurrentState === pathState.empty) ? true : false


    loadPathDialog.onAccepted: {
            map.deletePath()
            var path = loadPathDialog.fileUrl.toString()
            // remove prefixed "file://"
            path = path.replace(/^(file:\/{2})/, "")
            // unescape html codes like '%23' for '#'
            var cleanPath = decodeURIComponent(path)

            // console.log("Loaded file path: %1".arg(cleanPath))
            if (cmdWrapper.loadPathFromFile(cleanPath)) {
                mapView.pathCurrentState = pathState.empty
                wpCommands.wpButtonText = "Finalize..."
                wpCommands.wpButtonHighlighted = true
            }
        }

    savePathDialog.onAccepted: {
            var path = savePathDialog.fileUrl.toString()
            // remove prefixed "file://"
            path = path.replace(/^(file:\/{2})/, "")
            // unescape html codes like '%23' for '#'
            var cleanPath = decodeURIComponent(path)

            cmdWrapper.savePathToFile(cleanPath)
        }
    }
