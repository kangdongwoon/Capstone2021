import QtQuick 2.9
import QtQuick.Window 2.3
import QtQuick.Controls 2.2
import QtQuick.Dialogs 1.2
import QtQml 2.2
import QtGraphicalEffects 1.0

ApplicationWindow {
    visible: true
    width: Screen.width // 1920
    height: Screen.height // 1080
    title: qsTr("Trailer Reverse Assistance System")

    SwipeView{
        id: swipeView
        orientation: Qt.Vertical
        anchors.fill: parent
        currentIndex: tabBar.currentIndex

        TrailerUI {
            id: trailerUI
            Text {
                id: element
                x: 580
                y: 54
                width: 1308
                height: 132
                color: "#ffffff"
                text: qsTr("Trailer Reverse Assisstance System")
                anchors.horizontalCenterOffset: 0
                fontSizeMode: Text.FixedSize
                verticalAlignment: Text.AlignVCenter
                horizontalAlignment: Text.AlignHCenter
                anchors.horizontalCenter: parent.horizontalCenter
                font.pixelSize: 80
            }
            DropShadow {
                    anchors.fill: speeddigit
                    horizontalOffset: 0
                    verticalOffset: 8
                    radius: 4.0
                    samples: 16
                    color: "darkgray"
                    source: speeddigit
                }
        }

        Status {
        }
    }

    footer: TabBar {
        id: tabBar
        currentIndex: swipeView.currentIndex

        TabButton {
            text: qsTr("Trailer UI")
        }
        TabButton {
            text: qsTr("Status")
        }
    }
}
