import QtQuick 2.12
import QtQuick.Controls 2.12
import Qt.labs.platform 1.1

Item {
    id: root

    // Exposed properties
    property alias fileUrl: fileDialog.file
    property alias filePath: fileDialog.currentFile
    property alias folder: fileDialog.folder
    property bool open: false
    property string title: "Select a file"
    property var nameFilters: ["All Files (*)"]
    property bool selectFolder: false
    property bool saveFile: false           // <-- new
    property string defaultSuffix: ""       // <-- new (e.g., "txt")

    signal accepted(url file)
    signal rejected()

    FileDialog {
        id: fileDialog
        title: root.title
        nameFilters: root.nameFilters
        folder: StandardPaths.writableLocation(StandardPaths.HomeLocation) //  "file://" + home_dir + "/"//
        defaultSuffix: root.defaultSuffix   // <-- pass through
        fileMode: selectFolder
            ? FileDialog.Directory
            : saveFile
                ? FileDialog.SaveFile       // <-- save mode
                : FileDialog.OpenFile

        onAccepted: {
            root.accepted(fileDialog.file)
            root.open = false
        }

        onRejected: {
            root.rejected()
            root.open = false
        }
    }

    onOpenChanged: {
        if (open)
            fileDialog.open()
    }

    Component.onCompleted: {
        fileDialog.folder = StandardPaths.writableLocation(StandardPaths.HomeLocation)
    }
}