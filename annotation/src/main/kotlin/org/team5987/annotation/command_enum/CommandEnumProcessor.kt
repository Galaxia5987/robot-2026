package org.team5987.annotation.command_enum

import com.google.devtools.ksp.processing.*
import com.google.devtools.ksp.symbol.ClassKind
import com.google.devtools.ksp.symbol.KSAnnotated
import com.google.devtools.ksp.symbol.KSClassDeclaration
import com.squareup.kotlinpoet.*
import com.squareup.kotlinpoet.ksp.writeTo

const val ANNOTATION_PACKAGE = "org.team5987.annotation.command_enum.CommandEnum"

val snakeRegex = "_[a-zA-Z]".toRegex()

fun String.snakeToCamelCase(): String {
    return snakeRegex.replace(lowercase()) {
        it.value.replace("_","")
            .uppercase()
    }
}

class CreateCommandProcessor(
    env: SymbolProcessorEnvironment
) : SymbolProcessor {
    private val code = env.codeGenerator

    override fun process(resolver: Resolver): List<KSAnnotated> {
        val symbols = resolver.getSymbolsWithAnnotation(ANNOTATION_PACKAGE)

        symbols
            .filterIsInstance<KSClassDeclaration>()
            .filter { it.classKind == ClassKind.ENUM_CLASS }
            .forEach { enumDecl ->
                generateForEnum(enumDecl)
            }

        return emptyList()
    }

    fun generateInterface(
        pkg: String,
        fileName: String,
        enumName: String,
        entries: List<String>
    ): FileSpec {
        val enumClass = ClassName(pkg, enumName)
        val commandClass = ClassName("edu.wpi.first.wpilibj2.command", "Command")

        // generate all default entry functions
        val entryFunctions = entries.map { entry ->
            FunSpec.builder(entry.snakeToCamelCase())
                .returns(commandClass)
                .addStatement("return setTarget(%T.%L)", enumClass, entry)
                .build()
        }

        // abstract setTarget function
        val setTargetFun = FunSpec.builder("setTarget")
            .addParameter("value", enumClass)
            .returns(commandClass)
            .addModifiers(KModifier.ABSTRACT)
            .build()

        // the interface
        val interfaceSpec = TypeSpec.interfaceBuilder(fileName)
            .addFunctions(entryFunctions)
            .addFunction(setTargetFun)
            .build()

        // final file
        return FileSpec.builder(pkg, fileName)
            .addType(interfaceSpec)
            .build()
    }


    private fun generateForEnum(enumDecl: KSClassDeclaration) {
        val pkg = enumDecl.packageName.asString()
        val enumName = enumDecl.simpleName.asString()

        val entries = enumDecl.declarations
            .filterIsInstance<KSClassDeclaration>()
            .filter { it.classKind == ClassKind.ENUM_ENTRY }
            .map { it.simpleName.asString() }.toList()

        val fileName = "${enumName}CommandFactory"

        val generated: FileSpec = generateInterface(pkg, fileName, enumName, entries)

        generated.writeTo(code, Dependencies(false))
    }
}

class CommandEnumProcessorProvider : SymbolProcessorProvider {
    override fun create(environment: SymbolProcessorEnvironment): SymbolProcessor {
        return CreateCommandProcessor(environment)
    }
}