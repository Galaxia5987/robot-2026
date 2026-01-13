package frc.robot.lib.extensions

fun Class<*>.toPrimitiveTypeJava(): Class<*>? {
    return when (this) {
        java.lang.Integer::class.java -> Int::class.javaPrimitiveType
        java.lang.Boolean::class.java -> Boolean::class.javaPrimitiveType
        java.lang.Double::class.java -> Double::class.javaPrimitiveType
        java.lang.Float::class.java -> Float::class.javaPrimitiveType
        java.lang.Long::class.java -> Long::class.javaPrimitiveType
        java.lang.Short::class.java -> Short::class.javaPrimitiveType
        java.lang.Byte::class.java -> Byte::class.javaPrimitiveType
        java.lang.Character::class.java -> Char::class.javaPrimitiveType
        else -> this
    }
}
