package org.team5987.annotation

@Retention(AnnotationRetention.RUNTIME)
@Target(AnnotationTarget.PROPERTY, AnnotationTarget.FIELD, AnnotationTarget.FUNCTION, AnnotationTarget.PROPERTY_GETTER,
    AnnotationTarget.CLASS)
annotation class LoggedOutput(val level: LogLevel, val key: String = "", val path: String = "")
