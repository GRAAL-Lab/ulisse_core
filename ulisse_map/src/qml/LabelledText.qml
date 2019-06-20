import QtQuick 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1

LabelledTextForm {
    width: parent.width
    labelMouseArea.onClicked: {
        fbkUpdater.copyToClipboard(text)
        toast.show("Copied to clipboard")
    }
}
