var component;
var sprite;
var coordinates;

function createFlag(flagCoord) {
    coordinates = flagCoord;
    component = Qt.createComponent("qrc:/qml/WaypointFlag.qml");
    if (component.status === Component.Ready)
        finishCreation(coordinates);
    else
        component.statusChanged.connect(finishCreation);
}

function finishCreation() {
    if (component.status === Component.Ready) {
        sprite = component.createObject(map, {"coordinate:": coordinates});
        console.log("Flag Created (%1, %2)".arg(coordinates.latitude).arg(coordinates.longitude));
        if (sprite === null) {
            // Error Handling
            console.log("Error creating object");
        }
    } else if (component.status === Component.Error) {
        // Error Handling
        console.log("Error loading component:", component.errorString());
    }
}
