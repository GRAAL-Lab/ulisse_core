import QtQuick 2.6
import QtQuick.Controls 2.1

Dialog {
    property string dialogTitle: "Title"
    width: parent.width / 2
    x: (parent.width - width) / 2
    y: (parent.height - height) / 2
    modal: true
    standardButtons: Dialog.Ok
    title: dialogTitle
    onAccepted: {
        close()
    }
}
