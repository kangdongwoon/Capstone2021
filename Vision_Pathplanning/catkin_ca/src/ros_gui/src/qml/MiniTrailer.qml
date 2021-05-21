import QtQuick 2.4
import QtGraphicalEffects 1.0

Canvas {
    id: canvas

    property int value : 0

    onValueChanged: {zeiger.rotation = Math.min(Math.max(-45, canvas.value*3.5 - 0), 45); canvas.currentValue = zeiger.rotation - 0}
    //45 minrotation, -45 maxrotation
    width: parent.width; height: parent.height


    Rectangle {
        id: zeiger
        rotation: 0
        width: 4
        height: 25
        objectName: "minitrailer_yaw"
        transformOrigin: Item.Bottom
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.verticalCenter

        antialiasing: true
        smooth: true
        color: "#0c7104"
        onRotationChanged: {canvas.currentValue = zeiger.rotation - 0;}

        Rectangle {
            id: trailer
            x: -23
            y: -92
            width: 50
            height: 55
            color: "#a4a4a4"
            anchors.bottom: parent.top
            anchors.bottomMargin: 0
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.horizontalCenterOffset: 0
            z: 0
        }
    }

    Rectangle {
        id: link
        x: 178
        y: 320
        width: 4
        height: 20
        color: "#a4a4a4"
        anchors.verticalCenterOffset: 10
        anchors.verticalCenter: parent.verticalCenter
        anchors.horizontalCenter: parent.horizontalCenter

        Rectangle {
            id: hunter
            x: 319
            width: 50
            height: 76
            color: "#a4a4a4"
            anchors.top: parent.bottom
            anchors.topMargin: 0
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.horizontalCenterOffset: 0
            z: 0

            MiniTrailerWheel{
                objectName: "minitrailer_leftwheel"
                id: minitrailer_leftwheel
                x: 37
                y: 48
                width: 25
                height: 53
                anchors.verticalCenterOffset: 23
                anchors.verticalCenter: parent.verticalCenter
            }

            MiniTrailerWheel{
                objectName: "minitrailer_rightwheel"
                id: minitrailer_rightwheel
                x: -13
                y: 48
                width: 25
                height: 53
                anchors.verticalCenterOffset: 23
                anchors.verticalCenter: parent.verticalCenter
            }


        }
    }


    antialiasing: true

    property color secondaryColor: zeiger.color

    property real centerWidth: width / 2
    property real centerHeight: height / 2
    property real radius: Math.min(canvas.width, canvas.height) / 2

    property real minimumValue: -360
    property real maximumValue: 0
    property real currentValue: 0

    // this is the angle that splits the circle in two arcs
    // first arc is drawn from 0 radians to angle radians
    // second arc is angle radians to 2*PI radians
    property real angle: (currentValue - minimumValue) / (maximumValue - minimumValue) * 2 * Math.PI + 0.0001
    property real angleOffset: 4.71 // 270 degree //Math.Pi







}






























/*##^## Designer {
    D{i:2;anchors_height:75;anchors_y:20}D{i:1;anchors_height:20;anchors_y:56}
}
 ##^##*/
