import QtQuick 2.9
import QtQuick.Window 2.3
import QtQuick.Controls 2.2
import QtQml 2.2

ApplicationWindow {
    visible: true
    width: 1920
    height: 1080
    title: qsTr("Tocabi Controller")

    //    SwipeView {
    //        id: swipeView2
    //        anchors.fill: parent
    //        currentIndex: tabBar.currentIndex

    SwipeView{
        id: swipeView
        anchors.fill: parent
        currentIndex: tabBar.currentIndex

        Status {
        }
        Trailer{
        }
    }

    footer: TabBar {
        id: tabBar
        currentIndex: swipeView.currentIndex

        TabButton {
            text: qsTr("Status")
        }
        TabButton {
            text: qsTr("Trailer")
        }

    }
}
