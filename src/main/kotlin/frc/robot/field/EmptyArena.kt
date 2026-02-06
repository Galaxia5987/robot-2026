package frc.robot.field

import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.gamepieces.GamePiece

class EmptyArena : SimulatedArena(EmptyFieldObstacleMap()) {

    class EmptyFieldObstacleMap : FieldMap()
    override fun placeGamePiecesOnField() {}

    @Synchronized
    override fun getGamePiecesByType(type: String): MutableList<GamePiece>? =
        super.getGamePiecesByType(type)

    @Synchronized override fun clearGamePieces() {}
}
