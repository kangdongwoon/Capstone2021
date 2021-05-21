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
        width: 12
        height: 30
        objectName: "minitrailer_wheel"
        transformOrigin: Item.Center
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.verticalCenter

        antialiasing: true
        smooth: true
        color: "#0c7104"
        anchors.bottomMargin: -15
        onRotationChanged: {canvas.currentValue = zeiger.rotation - 0;}
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
    D{i:1;anchors_height:20;anchors_y:56}
}
 ##^##*/
