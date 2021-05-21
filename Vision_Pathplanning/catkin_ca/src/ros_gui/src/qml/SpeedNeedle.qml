import QtQuick 2.4
import QtGraphicalEffects 1.0

Canvas {
    id: canvas

    property int value : 0

    onValueChanged: {zeiger.rotation = Math.min(Math.max(-135, canvas.value*3.5 - 0), 135); canvas.currentValue = zeiger.rotation - 0}
                    //135 minrotation, -135 maxrotation
    width: parent.width; height: parent.height


    Rectangle {
        id: zeiger
        objectName: "racingwheel"
        rotation: 0
        width: 4
        height: parent.width / 2 * 0.9
        transformOrigin: Item.Bottom
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.verticalCenter

        antialiasing: true
        smooth: true
        color: "#0019f5"
        onRotationChanged: {canvas.currentValue = zeiger.rotation - 0; canvas.requestPaint()}
    }


    antialiasing: true

    property color secondaryColor: zeiger.color

    property real centerWidth: width / 2
    property real centerHeight: height / 2
    property real radius: Math.min(canvas.width*0.9, canvas.height*0.9) / 2

    property real minimumValue: -360
    property real maximumValue: 0
    property real currentValue: 0

    // this is the angle that splits the circle in two arcs
    // first arc is drawn from 0 radians to angle radians
    // second arc is angle radians to 2*PI radians
    property real angle: (currentValue - minimumValue) / (maximumValue - minimumValue) * 2 * Math.PI + 0.0001
    property real angleOffset: 4.71 // 270 degree //Math.Pi


    onPaint: {
    //Magic
        var ctx = getContext("2d");
        ctx.save();

        var gradient2 = ctx.createRadialGradient((parent.width / 2),(parent.height / 2), 500, (parent.width / 2),(parent.height / 2),5);
        gradient2.addColorStop(0.5, "#00095c");         //oben
        gradient2.addColorStop(0.46, "#5c6cff");        //mitte
        gradient2.addColorStop(0.33, "transparent");    //unten

        ctx.clearRect(0, 0, canvas.width, canvas.height);

        ctx.beginPath();
        ctx.lineWidth = 150;
        ctx.strokeStyle = gradient2
        ctx.arc(canvas.centerWidth, canvas.centerHeight, canvas.radius - (ctx.lineWidth / 2), canvas.angleOffset, canvas.angleOffset + canvas.angle, true);
        ctx.stroke();

        ctx.restore();

    }
}

