import QtQuick 2.0

Item {
    width: 960
    height: 540
    Image {
        id: image_streaming
        objectName: "cam_streaming"
        anchors.fill: parent
        cache: false

        fillMode: Image.PreserveAspectFit
        Timer {
            interval: 1
            repeat: true
            running: true
            onTriggered: {
                image_streaming.source = ""
                image_streaming.source = "image://rosimage/image_raw/streaming"
            }
        }
    }
}
