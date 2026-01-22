package org.team5987.annotation.machine_states

import com.google.devtools.ksp.processing.*
import com.google.devtools.ksp.symbol.ClassKind
import com.google.devtools.ksp.symbol.KSAnnotated
import com.google.devtools.ksp.symbol.KSClassDeclaration
import com.squareup.kotlinpoet.*
import com.squareup.kotlinpoet.ParameterizedTypeName.Companion.parameterizedBy
import com.squareup.kotlinpoet.ksp.writeTo

class MachineStatesProcessor(
    private val environment: SymbolProcessorEnvironment
) : SymbolProcessor {

    // You can change this to the fully qualified name of your specific annotation
    private val annotationName = "org.team5987.annotation.machine_states.MachineStates"

    override fun process(resolver: Resolver): List<KSAnnotated> {
        val symbols = resolver.getSymbolsWithAnnotation(annotationName)

        symbols
            .filterIsInstance<KSClassDeclaration>()
            .filter { it.classKind == ClassKind.ENUM_CLASS }
            .forEach { enumDecl ->
                generateStateMachine(enumDecl)
            }

        return emptyList()
    }

    private fun generateStateMachine(enumDecl: KSClassDeclaration) {
        val packageName = enumDecl.packageName.asString()
        val enumName = enumDecl.simpleName.asString()

        val enumClassName = ClassName(packageName, enumName)
        val stateMachineName = "${enumName}Machine"
        val stateMachineInterface = ClassName(packageName, stateMachineName)
        val triggerClass = ClassName("edu.wpi.first.wpilibj2.command.button", "Trigger")
        val commandClass = ClassName("edu.wpi.first.wpilibj2.command", "Command")
        val runOnceFun = MemberName("edu.wpi.first.wpilibj2.command.Commands","runOnce")
        val concurrentHashMapClass = ClassName("java.util.concurrent", "ConcurrentHashMap")

        val contextType = LambdaTypeName.get(receiver = triggerClass, returnType = UNIT)

        val enumProvider = LambdaTypeName.get(returnType = enumClassName)
        val pairType = ClassName("kotlin", "Pair").parameterizedBy(triggerClass, enumProvider)
        val transitionsArrayType = ClassName("kotlin", "Array").parameterizedBy(
            WildcardTypeName.producerOf(pairType)
        )

        val firstEntry = enumDecl.declarations
            .filterIsInstance<KSClassDeclaration>()
            .firstOrNull { it.classKind == ClassKind.ENUM_ENTRY }
            ?.simpleName?.asString()
            ?: enumDecl.declarations.firstOrNull()?.simpleName?.asString() ?: "entries.first()"

        val interfaceSpec = TypeSpec.interfaceBuilder(stateMachineInterface)
            .addProperty("context", contextType)
            .addProperty("transitions", transitionsArrayType)
            .addType(
                TypeSpec.companionObjectBuilder()
                    .addProperty(
                        PropertySpec.builder("currentState", enumClassName)
                            .mutable(true)
                            .initializer("%T.%L", enumClassName, firstEntry)
                            .build()
                    )
                    .build()
            )
            .build()

        val fileSpec = FileSpec.builder(packageName, stateMachineName)
            .addType(interfaceSpec)

        val enumWildcard = ClassName("kotlin", "Enum").parameterizedBy(STAR)
        val mapType = concurrentHashMapClass.parameterizedBy(enumWildcard, triggerClass)

        val cacheProperty = PropertySpec.builder("triggerCache", mapType)
            .addModifiers(KModifier.PRIVATE)
            .initializer("%T()", concurrentHashMapClass)
            .build()

        fileSpec.addProperty(cacheProperty)

        val stateTriggerFun = FunSpec.builder("stateTrigger")
            .receiver(enumClassName)
            .returns(triggerClass)
            .addCode(
                CodeBlock.builder()
                    .beginControlFlow("return triggerCache.getOrPut(this)")
                    .addStatement("%T { this == %T.currentState }", triggerClass, stateMachineInterface)
                    .indent()
                    .addStatement(".apply(this.context)")
                    .add(".apply {\n")
                    .indent()
                    .beginControlFlow("transitions.forEach { (cond, next) ->")
                    .addStatement("and(cond).onTrue(next().set())")
                    .endControlFlow()
                    .unindent()
                    .addStatement("}")
                    .unindent()
                    .endControlFlow()
                    .build()
            )
            .build()

        fileSpec.addFunction(stateTriggerFun)

        val setFun = FunSpec.builder("set")
            .receiver(enumClassName)
            .returns(commandClass)
            .addStatement("return %M ({ %T.currentState = this })", runOnceFun, stateMachineInterface)
            .build()

        fileSpec.addFunction(setFun)

        val invokeFun = FunSpec.builder("invoke")
            .addModifiers(KModifier.OPERATOR)
            .receiver(enumClassName)
            .returns(triggerClass)
            .addStatement("return stateTrigger()")
            .build()

        fileSpec.addFunction(invokeFun)

        fileSpec.build().writeTo(environment.codeGenerator, Dependencies(true, enumDecl.containingFile!!))
    }
}

class MachineStatesProcessorProvider : SymbolProcessorProvider {
    override fun create(environment: SymbolProcessorEnvironment): SymbolProcessor {
        return MachineStatesProcessor(environment)
    }
}
