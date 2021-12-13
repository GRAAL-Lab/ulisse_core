import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import "."

Rectangle {
    property var labelsize: 12
    property var textsize: 12
    property real panesMargin: 14
    property real panesWidth: 252
    property bool responsive: false

    color: Material.background

    GridLayout {
        id: gridView
        anchors.fill: parent
        columnSpacing: 0
        rowSpacing: 1
        rows: 2
        columns: 2

        Pane {
            id: statusPane
            Layout.rowSpan: 1
            Layout.columnSpan: 1
            Layout.fillWidth: true
            Layout.preferredWidth: gridView.width / 2
            Layout.leftMargin: 20

            ColumnLayout {
                id: statusdata
                width: parent.width
                Layout.fillHeight: true
                spacing: 8


                Label {
                    Layout.fillHeight: true
                    Layout.alignment: Qt.AlignHCenter
                    font.pointSize: 18
                    font.weight: Font.DemiBold
                    color: blue
                    text: "Status"
                }

                LabelledText {
                    Layout.fillHeight: true
                    Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                    labelColor: blue
                    textColor: 'grey'
                    text: fbkUpdater.vehicle_state
                    textBoldness: Font.Bold
                    label: "Vehicle State"
                    lsize: labelsize
                    tsize: textsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: blue
                    label: "GPS Time"
                    textColor: 'grey'
                    text: fbkUpdater.gps_time
                    lsize: labelsize
                    tsize: textsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: blue
                    label: "GPS Position"
                    textColor: 'grey'
                    text: "%1, %2".arg(fbkUpdater.gps_pos.latitude.toFixed(8)).arg(
                              fbkUpdater.gps_pos.longitude.toFixed(8))
                    lsize: labelsize
                    tsize: textsize
                }
            }
        }


        Pane {
            id: navFilterPane
            Layout.rowSpan: 1
            Layout.columnSpan: 1
            Layout.fillWidth: true
            Layout.preferredWidth: gridView.width / 2
            Layout.leftMargin: 20

            ColumnLayout {
                id: navFilterData
                width: parent.width
                Layout.fillHeight: true
                spacing: 8


                Label {
                    Layout.fillHeight: true
                    Layout.alignment: Qt.AlignHCenter
                    font.pointSize: 18
                    font.weight: Font.DemiBold
                    color: blue
                    text: "Navigation Filter"
                }


                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: blue
                    label: "Position"
                    textColor: 'grey'
                    text: "%1, %2, %3".arg(fbkUpdater.ulisse_pos.latitude.toFixed(8)).arg(
                              fbkUpdater.ulisse_pos.longitude.toFixed(8)).arg(fbkUpdater.ulisse_pos.altitude.toFixed(8))
                    lsize: labelsize
                    tsize: textsize
                }


                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: blue
                    label: "Linear Vel. (m/s)"
                    textColor: 'grey'
                    text: "%1,  %2,  %3".arg(fbkUpdater.ulisse_linear_vel.x.toFixed(2)).arg(
                            fbkUpdater.ulisse_linear_vel.y.toFixed(2)).arg(fbkUpdater.ulisse_linear_vel.z.toFixed(2))
                    lsize: labelsize
                    tsize: textsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: blue
                    label: "Orientation (°)"
                    textColor: 'grey'
                    text: "%1,  %2,  %3".arg(fbkUpdater.ulisse_rpy_deg.x.toFixed(2)).arg(
                              fbkUpdater.ulisse_rpy_deg.y.toFixed(2)).arg(fbkUpdater.ulisse_rpy_deg.z.toFixed(2))
                    lsize: labelsize
                    tsize: textsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: blue
                    label: "Angular Vel. (°/s)"
                    textColor: 'grey'
                    text: "%1,  %2,  %3".arg(fbkUpdater.ulisse_rpy_rate_deg.x.toFixed(2)).arg(
                              fbkUpdater.ulisse_rpy_rate_deg.y.toFixed(2)).arg(fbkUpdater.ulisse_rpy_rate_deg.z.toFixed(2))
                    lsize: labelsize
                    tsize: textsize
                }

                Row {
                    Layout.fillWidth: true
                    LabelledText {
                        Layout.alignment: Qt.AlignHCenter
                        labelColor: blue
                        label: "Filter Sensors"
                        textColor: 'grey'
                        text: ""
                        lsize: labelsize
                        tsize: textsize
                    }

                    Row {
                        spacing: 6
                        Rectangle {
                            width: gpsInfo.contentWidth + 6
                            height: gpsInfo.contentHeight + 6
                            border.color: fbkUpdater.gps_online ? "green" : "red"
                            border.width: 1
                            radius: 5

                            Text {
                                id: gpsInfo
                                verticalAlignment: Text.AlignVCenter
                                horizontalAlignment: Text.AlignHCenter
                                anchors.fill:parent
                                font.pointSize: 8
                                color: fbkUpdater.gps_online ? "green" : "red"
                                text:  fbkUpdater.gps_online ? qsTr("GPS") : qsTr("GPS N/A")
                            }
                        }

                        Rectangle {
                            width: compassInfo.contentWidth + 6
                            height: compassInfo.contentHeight + 6
                            border.color: fbkUpdater.compass_online ? "green" : "red"
                            border.width: 1
                            radius: 5

                            Text {
                                id: compassInfo
                                verticalAlignment: Text.AlignVCenter
                                horizontalAlignment: Text.AlignHCenter
                                anchors.fill:parent
                                font.pointSize: 8
                                color: fbkUpdater.compass_online ? "green" : "red"
                                text:  fbkUpdater.compass_online ? qsTr("Compass") : qsTr("Compass N/A")
                            }
                        }

                        Rectangle {
                            width: imuInfo.contentWidth + 6
                            height: imuInfo.contentHeight + 6
                            border.color: fbkUpdater.imu_online ? "green" : "red"
                            border.width: 1
                            radius: 5

                            Text {
                                id: imuInfo
                                verticalAlignment: Text.AlignVCenter
                                horizontalAlignment: Text.AlignHCenter
                                anchors.fill:parent
                                font.pointSize: 8
                                color: fbkUpdater.imu_online ? "green" : "red"
                                text:  fbkUpdater.imu_online ? qsTr("IMU") : qsTr("IMU N/A")
                            }
                        }

                        Rectangle {
                            width: magnetometerInfo.contentWidth + 6
                            height: magnetometerInfo.contentHeight + 6
                            border.color: fbkUpdater.magnetometer_online ? "green" : "red"
                            border.width: 1
                            radius: 5

                            Text {
                                id: magnetometerInfo
                                verticalAlignment: Text.AlignVCenter
                                horizontalAlignment: Text.AlignHCenter
                                anchors.fill:parent
                                font.pointSize: 8
                                color: fbkUpdater.magnetometer_online ? "green" : "red"
                                text:  fbkUpdater.magnetometer_online ? qsTr("Magnetometer") : qsTr("Magnetometer N/A")
                            }
                        }
                    }
                }
            }
        }

        Pane {
            id: lowLevelPane
            Layout.rowSpan: 1
            Layout.columnSpan: 1
            Layout.fillWidth: true
            Layout.preferredWidth: gridView.width / 2
            Layout.leftMargin: 20

            ColumnLayout {
                id: lowLevelData
                width: parent.width
                Layout.alignment: Qt.AlignHCenter
                Label {
                    Layout.alignment: Qt.AlignHCenter
                    font.pointSize: 18
                    font.weight: Font.DemiBold
                    color: darkgrey
                    text: "Low Level"
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: darkgrey
                    label: "Micro Loop Count"
                    textColor: 'grey'
                    text: "%1".arg(fbkUpdater.micro_loop_count)
                    lsize: labelsize
                    tsize: textsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: darkgrey
                    label: "Battery"
                    textColor: 'grey'
                    text: "Left: %1 \%, Right: %2 \%"
                    .arg(fbkUpdater.battery_perc_L).arg(fbkUpdater.battery_perc_R)
                    lsize: labelsize
                    tsize: textsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: darkgrey
                    label: "SW 485 Status"
                    textColor: 'grey'
                    text: "timestamp: "//.arg(fbkUpdater.right_satellite_received485)
                    lsize: labelsize
                    tsize: textsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: darkgrey
                    label: "Ambient"
                    textColor: 'grey'
                    text: "Temperature: %1 °C \nHumidity: %2 \%"
                    .arg(fbkUpdater.ambient_temperature.toFixed(1)).arg(fbkUpdater.ambient_humidity.toFixed(1))
                    lsize: labelsize
                    tsize: textsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: darkgrey
                    label: "Compass"
                    textColor: 'grey'
                    text: "RPY: %1".arg(fbkUpdater.compass_RPY)
                    lsize: labelsize
                    tsize: textsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: darkgrey
                    label: "IMU"
                    textColor: 'grey'
                    text: "Accelerometer: %1 \nGyroscope: %2".arg(fbkUpdater.imu_accelerometer).arg(fbkUpdater.imu_gyro)
                    lsize: labelsize
                    tsize: textsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: darkgrey
                    label: "Magnetometer"
                    textColor: 'grey'
                    text: "Strength: %1".arg(fbkUpdater.magnetometer)
                    lsize: labelsize
                    tsize: textsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: darkgrey
                    label: "Motors Speed"
                    textColor: 'grey'
                    text: "Left: %1, Right: %2 (rpm)".arg(fbkUpdater.motor_speed_L).arg(fbkUpdater.motor_speed_R)
                    lsize: labelsize
                    tsize: textsize
                }
            }
        }

        Pane {
            id: dclDataPane
            Layout.rowSpan: 1
            Layout.columnSpan: 1
            Layout.fillWidth: true
            Layout.preferredWidth: gridView.width / 2
            Layout.leftMargin: 20

            ColumnLayout {
                id: dclData
                width: parent.width
                Layout.fillHeight: true
                spacing: 8


                Label {
                    Layout.fillHeight: true
                    Layout.alignment: Qt.AlignHCenter
                    font.pointSize: 18
                    font.weight: Font.DemiBold
                    color: blue
                    text: "DCL"
                }


                GridLayout {
                    id: dclDataGrid
                    columns: 4
                    rows: 3

                    /// 1st ROW ///
                    Text { }
                    Text { text: "Feedback"; font.bold: true; }
                    Text { text: "Reference"; font.bold: true; }
                    Text { text: "Error"; font.bold: true; }


                    /// 2nd ROW ///
                    Text {
                        Layout.alignment: Qt.AlignHCenter
                        color: blue
                        text: "Surge (m/s)"
                        font.pointSize: labelsize
                        font.weight: Font.DemiBold
                        Layout.preferredWidth: 140
                    }
                    Text { text: "%1".arg(fbkUpdater.ulisse_surge); color: grey; Layout.preferredWidth: 120 }
                    Text { text: "%1".arg(fbkUpdater.desired_surge); color: grey; Layout.preferredWidth: 120 }
                    Text { text: "%1".arg((fbkUpdater.desired_surge - fbkUpdater.ulisse_surge)); color: grey; Layout.preferredWidth: 120 }


                    /// 3rd ROW ///
                    Text {
                        Layout.alignment: Qt.AlignHCenter
                        color: blue
                        text: "Yaw Rate (rad/s)"
                        font.pointSize: labelsize
                        font.weight: Font.DemiBold
                        Layout.preferredWidth: 140
                    }
                    Text { text: "%1".arg(fbkUpdater.ulisse_rpy_rate_deg.z); color: grey; Layout.preferredWidth: 120 }
                    Text { text: "%1".arg(fbkUpdater.desired_jog); color: grey; Layout.preferredWidth: 120 }
                    Text { text: "%1".arg((fbkUpdater.desired_jog - fbkUpdater.ulisse_rpy_rate_deg.z)); color: grey; Layout.preferredWidth: 120 }
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: blue
                    label: "Thrust. Ref."
                    textColor: 'grey'
                    text: "Left: %1 \%, Right: %2 \%"
                    .arg(fbkUpdater.thrust_ref_left).arg(fbkUpdater.thrust_ref_right)
                    lsize: labelsize
                    tsize: textsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: blue
                    label: "Thr. Applied Ref."
                    textColor: 'grey'
                    text: "Left: %1 \%, Right: %2 \%"
                    .arg(fbkUpdater.thrust_applied_ref_left).arg(fbkUpdater.thrust_applied_ref_right)
                    lsize: labelsize
                    tsize: textsize
                }
            }
        }
    }
}
