
package frc.robot.field

import edu.wpi.first.math.geometry.Translation2d
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.gamepieces.GamePiece

class EmptyArena : SimulatedArena(RapidReactFieldObstacleMap()) {

    class RapidReactFieldObstacleMap : FieldMap() {
        init {
            // blue wall
//            super.addBorderLine(
//                Translation2d(0.0, 1.270),
//                Translation2d(0.0, 6.782)
//            )

            // red wall
//            super.addBorderLine(
//                Translation2d(17.548, 1.270),
//                Translation2d(17.548, 6.782)
//            )

            // upper walls
//            super.addBorderLine(
//                Translation2d(1.672, 8.052),
//                Translation2d(17.548 - 1.672, 8.052)
//            )

            // lower walls
//            super.addBorderLine(
//                Translation2d(1.672, 0.0),
//                Translation2d(17.548 - 1.672, 0.0)
//            )
        }
    }
    override fun placeGamePiecesOnField() {}

    @Synchronized
    override fun getGamePiecesByType(type: String): MutableList<GamePiece>? =
        super.getGamePiecesByType(type)

    @Synchronized override fun clearGamePieces() {}
}
