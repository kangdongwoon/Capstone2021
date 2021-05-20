import QtQuick 2.9
import QtQuick.Window 2.3
import QtQuick.Controls 2.2
import QtQuick.Dialogs 1.2
import QtQml 2.2

Page {
    width: 1024
    height: 600
    title: qsTr("UI")

    Frame {
        id: frame5
        x: 613
        y: 261
        width: 384
        height: 226
        padding: 0

        Canvas {
            id: leftfoot
            objectName: "can1"
            x: -20
            y: 28
            width: 445
            height: 287
            onPaint: {
                var ctx = getContext("2d")
                ctx.fillStyle = Qt.rgba(1, 0, 0, 1)
                ctx.fillRect(ros.tt, 0, 50, 50)
            }

            Image {
                id: image
                x: 29
                y: 83
                width: 100
                height: 100
                //                source: "qrc:/qtquickplugin/images/template_image.png"
            }
        }

        Rectangle {
            id: rectangle
            objectName: "rec1"
            x: 207
            y: 52
            width: 28
            height: 26
            color: "#dcac9e"
            radius: 8
            z: 3
        }

        Rectangle {
            id: rectangle1
            objectName: "rec2"
            x: 207
            y: 156
            width: 28
            height: 26
            color: "#dcac9e"
            radius: 8
            z: 1
        }

        Rectangle {
            id: rectangle2
            objectName: "rec3"
            x: 200
            y: 144
            width: 130
            height: 50
            color: "#cbcbcb"
        }

        Rectangle {
            id: rectangle3
            objectName: "rec4"
            x: 200
            y: 40
            width: 130
            height: 50
            color: "#cbcbcb"
        }

        Rectangle {
            id: com1
            objectName: "com1"
            x: 104
            y: 29
            width: 16
            height: 16
            color: "#fd2121"
            radius: 8
            rotation: 0
            z: 4
        }
    }


    Timer{
        id:timer1
        interval: 16
        repeat:true
        running: true
        triggeredOnStart: true

        onTriggered: {
            ros.update();
        }
    }

    Text {
        id: text6
        x: 784
        y: 146
        width: 41
        height: 31
        text: qsTr("TIME")
        font.pixelSize: 18
    }

    Text {
        id: text7
        objectName: "time_text"
        x: 770
        y: 180
        width: 70
        height: 24
        text: qsTr("00:00:00")
        font.pixelSize: 18
    }

    Button {
        id: button
        x: 613
        y: 42
        text: qsTr("Gravity")
        onClicked: {
            ros.button_ros(0,"gravity");
        }
    }

    Button {
        id: button1
        x: 730
        y: 42
        text: qsTr("Task")
        onClicked: {
            ros.button_ros(1,"task");
        }
    }


    TabBar {
        id: tabBar
        x: 848
        y: 42
        width: 162
        height: 40
        currentIndex: swipeView.currentIndex


        TabButton {
            text: 'test 1 '
        }

        TabButton {
            text: 'test 2 '
        }
    }
}
