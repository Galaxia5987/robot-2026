package org.team5987.annotation

import com.google.devtools.ksp.processing.CodeGenerator
import com.google.devtools.ksp.processing.Dependencies
import com.google.devtools.ksp.processing.KSPLogger
import com.google.devtools.ksp.processing.Resolver
import com.google.devtools.ksp.processing.SymbolProcessor
import com.google.devtools.ksp.processing.SymbolProcessorEnvironment
import com.google.devtools.ksp.processing.SymbolProcessorProvider
import com.google.devtools.ksp.symbol.KSAnnotated
import com.google.devtools.ksp.symbol.KSClassDeclaration
import com.google.devtools.ksp.symbol.KSPropertyDeclaration
import com.google.devtools.ksp.symbol.KSTypeReference
import com.google.devtools.ksp.symbol.Modifier
import com.google.devtools.ksp.symbol.Origin
import com.google.devtools.ksp.validate
import com.squareup.kotlinpoet.ClassName
import com.squareup.kotlinpoet.FileSpec
import com.squareup.kotlinpoet.FunSpec
import com.squareup.kotlinpoet.KModifier
import com.squareup.kotlinpoet.TypeSpec
import com.squareup.kotlinpoet.ksp.toClassName
import com.squareup.kotlinpoet.ksp.writeTo

class LoggedProcessor(private val codeGenerator: CodeGenerator, private val logger: KSPLogger): SymbolProcessor {
    private val logTableType = ClassName("org.littletonrobotics.junction", "LogTable")
    private val loggableInputsType = ClassName("org.littletonrobotics.junction.inputs", "LoggableInputs")

    private fun isFromJava(propertyType: KSTypeReference): Boolean =
        propertyType.resolve().declaration.origin in setOf(Origin.JAVA_LIB, Origin.JAVA)

    private fun getPropertySuperClass(property: KSPropertyDeclaration): KSClassDeclaration? {
        val propertyType = property.type.resolve()
        return propertyType.declaration as? KSClassDeclaration
    }

    override fun process(resolver: Resolver): List<KSAnnotated> {
        val annotatedClasses = resolver.getSymbolsWithAnnotation("org.team5987.annotation.Logged").filterIsInstance<KSClassDeclaration>()
        annotatedClasses.forEach { process(it) }
        return annotatedClasses.filterNot { it.validate() }.toList()
    }

    private fun process(classDeclaration: KSClassDeclaration) {
        if (!classDeclaration.modifiers.contains(Modifier.OPEN)) throw Exception("""[Logged] Please ensure the class you are annotating (${classDeclaration.simpleName.asString()}) has the open modifier!""")

        val packageName = classDeclaration.packageName.asString()
        val className = classDeclaration.simpleName.asString()

        val newClassName = "Logged${className}"

        val toLogBuilder = FunSpec.builder("toLog")
            .addModifiers(KModifier.OVERRIDE)
            .addParameter("table", logTableType)
        val fromLogBuilder = FunSpec.builder("fromLog")
            .addModifiers(KModifier.OVERRIDE)
            .addParameter("table", logTableType)


        classDeclaration.getAllProperties().forEach { property ->
            val simpleName = property.simpleName.asString()
            val logName = simpleName.substring(0, 1).uppercase() + simpleName.substring(1)

            if (!property.isMutable) throw Exception("""[Logged] Please ensure the class you are annotating (${classDeclaration.simpleName.asString()}) has only mutable properties!""")

            toLogBuilder.addCode(
                """ |table.put("$logName", $simpleName)
                    |
                """.trimMargin()
            )

            var fromLogCode =
                """ |$simpleName = table.get("$logName", $simpleName)"""
            // If the type comes from java kotlin is not sure whether it's nullable or not
            if (isFromJava(property.type) && !getPropertySuperClass(property)?.toClassName().toString().lowercase().contains("measure")) fromLogCode += "!![0]!!" // Assume the type is not nullable

            fromLogCode = ("$fromLogCode\n|").trimMargin()

            fromLogBuilder.addCode(
                fromLogCode
            )
        }

        val type = TypeSpec.classBuilder(newClassName)
            .addSuperinterface(loggableInputsType)
            .superclass(classDeclaration.toClassName())
            .addFunction(toLogBuilder.build())
            .addFunction(fromLogBuilder.build())


        val file = FileSpec.builder(packageName, newClassName)
            .addType(type.build())
            .addImport("frc.robot.lib", "put")
            .addImport("frc.robot.lib", "get")
            .indent("    ")
            .build()
        file.writeTo(codeGenerator, Dependencies(true, classDeclaration.containingFile!!))
    }
}

class Provider: SymbolProcessorProvider {
    override fun create(environment: SymbolProcessorEnvironment) = LoggedProcessor(
        codeGenerator = environment.codeGenerator,
        logger = environment.logger
    )
}