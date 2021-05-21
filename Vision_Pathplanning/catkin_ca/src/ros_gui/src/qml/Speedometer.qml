import QtQuick 2.4
import QtGraphicalEffects 1.0

Rectangle {
    color: "transparent"

    Needle {
        id: needle
        anchors.verticalCenterOffset: 0
        anchors.centerIn: parent
        focus: true
    }

    SpeedNeedle {
        id: speedNeedle
        objectName: "racingwheel_value"
        anchors.verticalCenterOffset: 0
        anchors.centerIn: parent
        focus: true
    }


    InnerRing    {
        id: innerring
        y: 120
        speed: (Math.round(speedNeedle.currentValue, 0) + 0)//360) * 0.68
    }

    //Letter: P R N D
    Grid {
        y: 435
        anchors.horizontalCenter: parent.horizontalCenter
        columns: 4
        Rectangle { color: "transparent"; width: 25; height: 25
            Text {
                objectName: "letterP"
                id: letterP
                text: " P "
                font.family: "Eurostile"; font.pixelSize: 36
                color: "white"
                anchors.centerIn: parent
            } }
        Rectangle { color: "transparent"; width: 25; height: 25
            Text {
                objectName: "letterR"
                id: letterR
                text: " R "
                font.family: "Eurostile"; font.pixelSize: 18
                color: "darkgray"
                anchors.centerIn: parent
            }}
        Rectangle { color: "transparent"; width: 25; height: 25
            Text {
                objectName: "letterN"
                text: " N "
                font.family: "Eurostile"; font.pixelSize: 18
                color: "darkgray"
                anchors.centerIn: parent
            }}
        Rectangle { color: "transparent"; width: 25; height: 25
            Text {
                objectName: "letterD"
                id: letterD
                text: " D "
                font.family: "Eurostile"; font.pixelSize: 18
                color: "darkgray"
                anchors.centerIn: parent
            }}
    }

    function drive() {
        letterD.font.bold = true
        letterD.color = "white"
        letterD.font.pixelSize = 36
        letterR.font.bold = false
        letterR.color = "darkgray"
        letterR.font.pixelSize = 18
    }
    function back() {
        letterR.font.bold = true
        letterR.color = "white"
        letterR.font.pixelSize = 36
        letterD.font.bold = false
        letterD.color = "darkgray"
        letterD.font.pixelSize = 18
    }


}














/*##^## Designer {
    D{i:0;autoSize:true;height:480;width:640}
}
 ##^##*/
