package frc.robot.lib.extensions

import edu.wpi.first.networktables.NetworkTableInstance

// IMPORTANT: USE THIS ONLY IF YOU ALREADY ADDED THE KEY TO THE CHOOSER!!!
fun setLoggedAutoChooser(chooserName: String, key: String) {
    val table =
        NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getSubTable(chooserName)
    table.getEntry("selected").setString(key)
}
