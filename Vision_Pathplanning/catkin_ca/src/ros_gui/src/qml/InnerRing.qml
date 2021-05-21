import QtQuick 2.0
import QtGraphicalEffects 1.0

Item {

    property int speed: 0

    height: 335
    width: height
    x: (parent.width / 2) - (width / 2)
    y: (parent.height / 2) - (height / 2)

    Image {
         id: innerRingRect
         height: parent.height
         width: parent.width


         Text {
             id: speeddigit
             text: speed
             anchors.horizontalCenterOffset: 0
             font.pixelSize: 86
             font.bold: true
             font.family: "Eurostile"
             y: 74
             color: "white"
             anchors.horizontalCenter: parent.horizontalCenter
         }

        DropShadow {
                anchors.fill: speeddigit
                horizontalOffset: 0
                verticalOffset: 8
                radius: 4.0
                samples: 16
                color: "black"
                source: speeddigit
            }

         Text {
             text: "degree"
             anchors.horizontalCenterOffset: 0
             font.pixelSize: 16
             font.bold: true
             font.family: "Eurostile"
             y: 166
             color: "white"
             anchors.horizontalCenter: parent.horizontalCenter
         }

         Text {
             text: "Capstone"
             font.pixelSize: 34
             font.bold: true
             font.family: "Eurostile"
             y: 220
             color: "white"
             anchors.horizontalCenter: parent.horizontalCenter
         }

    }
}
