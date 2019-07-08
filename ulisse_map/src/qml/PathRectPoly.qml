import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.2

PathRectPolyForm {
   id: rowLayout1
    b_path.onClicked:{
                map.createPath()
            }
    b_rect.onClicked: {
                map.createRect()
            }
    b_poly.onClicked: {
                map.createPoly()
            }
}
