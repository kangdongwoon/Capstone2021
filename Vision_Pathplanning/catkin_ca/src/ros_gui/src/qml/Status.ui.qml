import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQml 2.2

Page {
    id: page
    width: 1920
    height: 1080
    clip: true
    wheelEnabled: true
    title: qsTr("Trailer Reverse Assistance System")

    Frame {
        id: frame
        x: 45
        y: 94
        width: 325
        height: 447

        Frame {
            id: frame4
            x: 10
            y: 34
            width: 281
            height: 91
            ProgressBar {
                id: p1
                x: 0
                y: 0
                from: -1
                objectName: "p1"
                value: 0
            }

            ProgressBar {
                id: p2
                x: 0
                y: 20
                from: -1
                objectName: "p2"
                value: 0
            }

            ProgressBar {
                id: p3
                x: 0
                y: 41
                from: -1
                objectName: "p3"
                value: 0
            }

            Text {
                id: t1
                x: 214
                y: -4
                text: qsTr("000.000")
                font.pixelSize: 12
                objectName: "t1"
                horizontalAlignment: Text.AlignRight
            }

            Text {
                id: t2
                x: 214
                y: 16
                text: qsTr("000.000")
                font.pixelSize: 12
                objectName: "t2"
                horizontalAlignment: Text.AlignRight
            }

            Text {
                id: t3
                x: 214
                y: 36
                text: qsTr("000.000")
                font.pixelSize: 12
                objectName: "t3"
                horizontalAlignment: Text.AlignRight
            }

            Text {
                id: text5
                x: 87
                text: qsTr("Quaternion")
                anchors.horizontalCenterOffset: 0
                anchors.top: parent.top
                anchors.topMargin: -40
                anchors.horizontalCenter: parent.horizontalCenter
                font.pixelSize: 16
            }

            ProgressBar {
                id: p4
                x: 0
                y: 61
                from: -1
                objectName: "p4"
                value: 0
            }

            Text {
                id: t4
                x: 214
                y: 57
                text: qsTr("000.000")
                horizontalAlignment: Text.AlignRight
                objectName: "t4"
                font.pixelSize: 12
            }
        }

        Frame {
            id: frame6
            x: 10
            y: 196
            width: 281
            height: 70
            ProgressBar {
                id: p5
                x: 0
                y: 0
                to: 10
                from: -10
                objectName: "p5"
                value: 0
            }

            ProgressBar {
                id: p6
                x: 0
                y: 21
                to: 10
                from: -10
                objectName: "p6"
                value: 0
            }

            ProgressBar {
                id: p7
                x: 0
                y: 41
                from: -10
                to: 10
                objectName: "p7"
                value: 0
            }

            Text {
                id: t5
                x: 214
                y: -4
                text: qsTr("000.000")
                horizontalAlignment: Text.AlignRight
                objectName: "t5"
                font.pixelSize: 12
            }

            Text {
                id: t6
                x: 214
                y: 16
                text: qsTr("000.000")
                horizontalAlignment: Text.AlignRight
                objectName: "t6"
                font.pixelSize: 12
            }

            Text {
                id: t7
                x: 214
                y: 36
                text: qsTr("000.000")
                horizontalAlignment: Text.AlignRight
                objectName: "t7"
                font.pixelSize: 12
            }

            Text {
                id: text8
                x: 93
                text: qsTr("Angular Velocity")
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.top: parent.top
                anchors.topMargin: -40
                font.pixelSize: 16
            }
        }

        Frame {
            id: frame7
            x: 10
            y: 340
            width: 281
            height: 69
            ProgressBar {
                id: p8
                x: 0
                y: 0
                to: 10
                from: -10
                objectName: "p8"
                value: 0
            }

            ProgressBar {
                id: p9
                x: 0
                y: 21
                to: 10
                from: -10
                objectName: "p9"
                value: 0
            }

            ProgressBar {
                id: p10
                x: 0
                y: 41
                to: 10
                from: -10
                objectName: "p10"
                value: 0
            }

            Text {
                id: t8
                x: 214
                y: -4
                text: qsTr("000.000")
                horizontalAlignment: Text.AlignRight
                objectName: "t8"
                font.pixelSize: 12
            }

            Text {
                id: t9
                x: 214
                y: 16
                text: qsTr("000.000")
                horizontalAlignment: Text.AlignRight
                objectName: "t9"
                font.pixelSize: 12
            }

            Text {
                id: t10
                x: 214
                y: 36
                text: qsTr("000.000")
                horizontalAlignment: Text.AlignRight
                objectName: "t10"
                font.pixelSize: 12
            }

            Text {
                id: text9
                x: 93
                text: qsTr("Linear Acceleration")
                anchors.top: parent.top
                anchors.topMargin: -40
                anchors.horizontalCenter: parent.horizontalCenter
                font.pixelSize: 16
            }
        }

        Text {
            id: element
            x: 139
            text: qsTr("IMU")
            anchors.top: parent.top
            anchors.topMargin: -75
            anchors.horizontalCenter: parent.horizontalCenter
            font.pixelSize: 50
        }
    }
    Frame {
        id: frame1
        x: 400
        y: 94
        width: 325
        height: 447
        Frame {
            id: frame5
            x: 10
            y: 34
            width: 281
            height: 91
            ProgressBar {
                id: lineargaze
                x: 62
                y: 36
                width: 138
                height: 9
                value: 0
                from: -1
                objectName: "lineargaze"
            }

            Text {
                id: car_xvalue
                x: 60
                y: 6
                text: qsTr("000.000")
                horizontalAlignment: Text.AlignRight
                font.pixelSize: 12
                objectName: "car_xvalue"
            }

            Text {
                id: car_yvalue
                x: 187
                y: 6
                text: qsTr("000.000")
                horizontalAlignment: Text.AlignRight
                font.pixelSize: 12
                objectName: "car_yvalue"
            }

            Text {
                id: linear_value
                x: 214
                y: 36
                text: qsTr("000.000")
                horizontalAlignment: Text.AlignRight
                font.pixelSize: 12
                objectName: "linear_value"
            }

            Text {
                id: car_status
                x: 87
                text: qsTr("Car Staus")
                anchors.horizontalCenterOffset: 0
                anchors.horizontalCenter: parent.horizontalCenter
                font.pixelSize: 16
                anchors.topMargin: -40
                anchors.top: parent.top
            }

            ProgressBar {
                id: steergaze
                x: 62
                y: 61
                width: 138
                height: 6
                value: 0
                from: -1
                objectName: "steergaze"
            }

            Text {
                id: steer_value
                x: 214
                y: 57
                text: qsTr("000.000")
                horizontalAlignment: Text.AlignRight
                font.pixelSize: 12
                objectName: "steer_value"
            }

            Text {
                id: car_x
                x: 38
                y: 5
                text: qsTr("x :")
                font.pixelSize: 12
            }

            Text {
                id: car_y
                x: 169
                y: 5
                text: qsTr("y :")
                font.pixelSize: 12
            }

            Text {
                id: car_linear
                x: 6
                y: 33
                text: qsTr("linear")
                font.pixelSize: 12
            }

            Text {
                id: car_steering
                x: -8
                y: 57
                text: qsTr("steerangle")
                font.pixelSize: 12
            }
        }

        Frame {
            id: frame8
            x: 10
            y: 166
            width: 281
            height: 54
            Text {
                id: trailer_xvalue
                x: 57
                y: 9
                text: qsTr("000.000")
                horizontalAlignment: Text.AlignRight
                font.pixelSize: 12
                objectName: "trailer_xvalue"
            }

            Text {
                id: trailer_yvalue
                x: 184
                y: 8
                text: qsTr("000.000")
                horizontalAlignment: Text.AlignRight
                font.pixelSize: 12
                objectName: "trailer_yvalue"
            }

            Text {
                id: trailer_status
                x: 93
                text: qsTr("Trailer Status")
                anchors.horizontalCenter: parent.horizontalCenter
                font.pixelSize: 16
                anchors.topMargin: -40
                anchors.top: parent.top
            }

            Text {
                id: trailer_y
                x: 165
                y: 7
                text: qsTr("y :")
                font.pixelSize: 12
            }

            Text {
                id: trailer_x
                x: 36
                y: 8
                text: qsTr("x :")
                font.pixelSize: 12
            }
        }

        Frame {
            id: frame9
            x: 10
            y: 359
            width: 281
            height: 50
            Text {
                id: lookahead_xvalue
                x: 53
                y: 6
                text: qsTr("000.000")
                horizontalAlignment: Text.AlignRight
                font.pixelSize: 12
                objectName: "lookahead_xvalue"
            }

            Text {
                id: lookahead_yvalue
                x: 182
                y: 6
                text: qsTr("000.000")
                horizontalAlignment: Text.AlignRight
                font.pixelSize: 12
                objectName: "lookahead_yvalue"
            }

            Text {
                id: lookahead_status
                x: 93
                text: qsTr("LookAhead Point")
                anchors.horizontalCenter: parent.horizontalCenter
                font.pixelSize: 16
                anchors.topMargin: -40
                anchors.top: parent.top
            }

            Text {
                id: lookahead_y
                x: 164
                y: 6
                text: qsTr("y :")
                font.pixelSize: 12
            }

            Text {
                id: lookahead_x
                x: 35
                y: 6
                text: qsTr("x :")
                font.pixelSize: 12
            }
        }

        Text {
            id: element6
            x: 131
            y: 0
            text: qsTr("Status")
            anchors.horizontalCenter: parent.horizontalCenter
            font.pixelSize: 50
            anchors.topMargin: -75
            anchors.top: parent.top
        }

        Frame {
            id: frame10
            x: 10
            y: 265
            width: 281
            height: 50
            Text {
                id: btwangle
                x: 107
                y: 6
                text: qsTr("000.000")
                horizontalAlignment: Text.AlignRight
                font.pixelSize: 12
                objectName: "btwangle"
            }

            Text {
                id: car_trailer_angle_status
                x: 93
                text: qsTr("Car - Trailer Angle")
                anchors.horizontalCenter: parent.horizontalCenter
                font.pixelSize: 16
                anchors.topMargin: -40
                anchors.top: parent.top
            }
        }
    }

    Dial {
        id: tar_steeringdial
        x: 753
        y: 139
        from: -135
        stepSize: 0.01
        value: 0
        to: 135
        enabled: true
        objectName: "tar_steeringdial"

        Text {
            id: target
            x: 8
            y: -39
            text: qsTr("Desired")
            anchors.horizontalCenterOffset: 0
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: parent.top
            font.pixelSize: 16
            anchors.topMargin: -34
        }
    }

    Dial {
        id: cur_steeringdial
        x: 753
        y: 326
        to: 135
        value: 0
        stepSize: 0.01
        objectName: "cur_steeringdial"
        from: -135
        enabled: true

        Text {
            id: current
            x: 8
            y: -43
            text: qsTr("Current")
            anchors.horizontalCenterOffset: 0
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: parent.top
            font.pixelSize: 16
            anchors.topMargin: -28
        }
    }
}
