import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Window 2.2
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.3

Rectangle {
    id: trailerui
    width: Screen.width // 1920
    height: Screen.height // 1080
    enabled: false
    clip: true
    color: "black"

    Item {
        Timer {
            interval: 10
            repeat: true
            running: true
            triggeredOnStart: true
            onTriggered: ros.update()
        }
    }

    Speedometer {
        x: 200
        y: 100
        width: 675
        height: 675
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 0
        anchors.right: parent.right
        anchors.rightMargin: 120
    }

    Cameras {
        id: cameras
        y: 300
        width: 960
        height: 540
        anchors.left: parent.left
        anchors.leftMargin: 60
        anchors.verticalCenter: parent.verticalCenter
    }

    MiniTrailer {
        id: minitrailer
        x: 1700
        y: 154
        width: 100
        height: 220
        rotation: 180
    }
}
